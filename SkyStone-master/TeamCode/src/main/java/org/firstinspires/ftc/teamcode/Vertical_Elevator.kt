package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Universal.Motion.MotionProfile
import org.openftc.revextensions2.RevBulkData
import kotlin.math.abs

class Vertical_Elevator(map : HardwareMap, t : Telemetry){
    var motors : Array<Caching_Motor>

    val UPDATE_RATE = 1
    var write_index = 0
    var dclickt = ElapsedTime()

    var target = 0
    var x = 1

    var telemetry = t
    var isDropped = false
    var touch = map.get(DigitalChannel::class.java, "touch")

    var zero = 0.0

    var time = ElapsedTime()
    var clicks = 0

    var stack_count = 0

    var TargetPos = arrayOf(10, 108, 200, 280, 360, 440, 515, 600, 680, 755, 836, 913)//arrayOf(10, 50, 113, 205, 285, 365, 445, 520, 605, 760, 841, 918)

    var fine_tune = 1.0
    var error = 0.0

    var lastError = 0.0

    var lastTime = System.currentTimeMillis()

    var holdTime = ElapsedTime()

    companion object{
        const val kp = 0.00875
        const val kd = 0.0
        const val FF = 0.0001
    }

    enum class slideState{
        STATE_RAISE_INCREMENT,
        STATE_RAISE,
        STATE_DROP,
        STATE_IDLE
    }

    enum class slideBoundary{
        STATE_LOW,
        STATE_OPTIMAL,
        STATE_HIGH,
        STATE_UNKNOWN
    }

    var mSlideState = slideState.STATE_IDLE

    init{
        motors = arrayOf(Caching_Motor(map, "lift_1"), Caching_Motor(map, "lift_2"))
        touch.mode = DigitalChannel.Mode.INPUT
        motors[1].motor.direction = DcMotorSimple.Direction.REVERSE
        motors.map{
            it.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        dclickt.startTime()
        holdTime.startTime()
    }

    fun PIDController(TargetLevel : Int){
        error = TargetPos[TargetLevel] - getLiftHeight()

        var prop_gain = kp * error
        var deriv_gain = kd * ((error - lastError) / (System.currentTimeMillis() - lastTime))
        var ff_gain = 0.0
        if (TargetLevel > 8) {
            ff_gain = FF * TargetPos[TargetLevel]
        }
        lastTime = System.currentTimeMillis()
        var power = prop_gain + deriv_gain + ff_gain

        if (abs(power) > 0.001) {
            setPower(power)
        }
        else{
            setPower(0.0)
        }
        lastError = error
    }

    fun read(data : RevBulkData){
        motors.map {
            it.read(data)
        }
        isDropped = !data.getDigitalInputState(touch)
    }

    fun write(){
        motors[write_index].write()
        write_index = (write_index + 1) % 2
    }

    fun setPower(power : Double){
        motors.map{
            it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        if(!isDropped){
            when(getBoundaryConditions()){
                slideBoundary.STATE_OPTIMAL ->
                    motors.map{ it.setPower(Range.clip(power + 0.3, -1.0, 1.0)) }
                slideBoundary.STATE_LOW ->
                    motors.map { it.setPower(Range.clip(power, -1.0, 1.0)) }
                slideBoundary.STATE_UNKNOWN ->
                    motors.map{it.setPower(Range.clip(power, -1.0, 1.0))}
            }
        }else if(isDropped){
            val newPower = Range.clip(power, 0.0, 1.0)
            motors.map{ it.setPower(newPower)}
            zero = getLiftHeight() //reset zero by changing perception of encoder values, as resetting the encoders through firmware counts as a hardware transaction, and is therefore inefficient. Think scale indices with a changing key
        }

        telemetry.addData("Speed Set", power)
    }

    fun getLiftHeight() : Double{
        return ((motors[0].getCurrentPosition() + motors[1].getCurrentPosition()) / 2).toDouble() - zero
    }

    fun getBoundaryConditions() : slideBoundary{
        if (getLiftHeight() >= 50){
            return slideBoundary.STATE_OPTIMAL
        }
        else if (getLiftHeight() < 100){
            return slideBoundary.STATE_LOW
        }
        else{
            return slideBoundary.STATE_UNKNOWN
        }
    }

    fun newState(s : slideState){
        mSlideState = s
    }

    fun increment_stack_count(){
        stack_count = (stack_count + 1) % 12
    }

    fun decrement_stack_count(){
        stack_count = Math.abs(stack_count - 1)
    }

    fun setTargetPosition(target : Int){
        motors[0].motor.targetPosition = target
        motors[1].motor.targetPosition = target//motors[0].getCurrentPosition() //test to see if pos1 = -pos2, recommended test is to output encoder positions from both motors and analyze

        motors.map{
            it.motor.mode = DcMotor.RunMode.RUN_TO_POSITION
            it.motor.power = 0.3
        }
    }

    fun setTargetPosBasic(target: Int, power: Double){
        motors.map {
            if(stack_count > 1){
                it.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                if(getLiftHeight() < (target)){
                    it.motor.power = power
                }else if(getLiftHeight() >= (target)) {
                    it.motor.power = 0.3
                }
            }else if(stack_count <= 1){
                it.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
                if(getLiftHeight() < target){
                    it.motor.power = power
                }else if(getLiftHeight() >= target) {
                    it.motor.power = 0.3
                }
            }
            if(power <= 0.0){
                telemetry.addData("State:", "Dropping")
                if(getLiftHeight() > (target)){
                    it.motor.power = power
                }else if(getLiftHeight() <= (target)) {
                    it.motor.power = 0.0
                }
            }
        }
    }

    fun operate(g : Gamepad){
        motors.map{
            it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        if (g.x){
            newState(slideState.STATE_RAISE)
        }
        else if (g.b){
            newState(slideState.STATE_DROP)
        }else if(g.dpad_up && dclickt.time() >= 0.3){
            increment_stack_count()
            newState(slideState.STATE_RAISE_INCREMENT)
            dclickt.reset()
        }else if(g.dpad_down && dclickt.time() >= 0.3){
            decrement_stack_count()
            newState(slideState.STATE_RAISE_INCREMENT)
            dclickt.reset()
        }


        if(mSlideState == slideState.STATE_RAISE_INCREMENT){
            newState(slideState.STATE_IDLE)
        }
        else if (mSlideState == slideState.STATE_RAISE) {
            fine_tune = 0.5
            PIDController(stack_count)
            if (abs(getLiftHeight() - TargetPos[stack_count]) > 20){
                holdTime.reset()
            }
            else{
                if (holdTime.time() >= 0.5) {
                    newState(slideState.STATE_IDLE)
                }
            }
            /*if(stack_count != 0){
                PIDController(stack_count - 1)
                if (abs(getLiftHeight() - TargetPos[stack_count - 1]) < 5){
                    newState(slideState.STATE_IDLE)
                }
            }else if(stack_count == 0){
                PIDController(stack_count)
                if (abs(getLiftHeight() - TargetPos[stack_count]) < 5){
                    newState(slideState.STATE_IDLE)
                }
            }*/
            write()
        }
        else if (mSlideState == slideState.STATE_DROP){
            fine_tune = 1.0
            setTargetPosBasic(6, -0.25)
            if(getLiftHeight() <= 2){
                newState(slideState.STATE_IDLE)
            }
        }
        else if (mSlideState == slideState.STATE_IDLE){
            if(g.right_stick_y < 0){
                setPower(-0.5 * g.right_stick_y)
            }else{
                setPower(-0.3 * g.right_stick_y)
            }

        }
        /*if (mSlideState != slideState.STATE_RAISE) {
            write()
        }*/
        write()
        telemetry.addData("Level:", stack_count)
        telemetry.addData("Height: ", getLiftHeight())
        telemetry.addData("Slide Motor 1 Target: ", target)
        telemetry.addData("Slide Motor 2 Target: ", target)
    }
}