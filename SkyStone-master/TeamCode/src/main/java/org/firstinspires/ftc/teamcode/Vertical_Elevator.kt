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

    var x = 1

    var telemetry = t
    var isDropped = false
    var touch = map.get(DigitalChannel::class.java, "touch")

    var zero = 0.0

    var time = ElapsedTime()
    var clicks = 0

    var stack_count = 0

    val NUM_SAFE_1x2 = 6
    val NUM_TOTAL = 6

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
                    motors.map{ it.setPower(power + 0.3) }
                slideBoundary.STATE_LOW ->
                    motors.map { it.setPower(power) }
                slideBoundary.STATE_UNKNOWN ->
                    motors.map{it.setPower(power)}
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
        if (getLiftHeight() >= 100){
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
        stack_count++
    }

    fun decrement_stack_count(){
        stack_count--
    }

    fun getTargetHeight() : Int{
        return when{
            stack_count >= (NUM_TOTAL - NUM_SAFE_1x2) -> stack_count - (NUM_TOTAL - NUM_SAFE_1x2)
            stack_count < (NUM_TOTAL - NUM_SAFE_1x2) -> (stack_count / 2)
            else -> 0
        }
    }

    fun setTargetPosition(target : Int){
        motors.map{
            it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        motors[0].motor.targetPosition = target
        motors[1].motor.targetPosition = target//motors[0].getCurrentPosition() //test to see if pos1 = -pos2, recommended test is to output encoder positions from both motors and analyze

        if (motors[0].motor.mode != DcMotor.RunMode.RUN_TO_POSITION){
            motors[0].motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        }
        if (motors[1].motor.mode != DcMotor.RunMode.RUN_TO_POSITION){
            motors[1].motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        }
        motors.map{
            it.motor.power = 1.0
        }
    }

    fun operate(g : Gamepad){
        motors.map{
            it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        if (g.a){
            //newState(slideState.STATE_RAISE)
        }
        else if (g.y){
            //newState(slideState.STATE_DROP)
        }else if(g.dpad_up && time.time() == 1.0){
            clicks += 1 % 12
            newState(slideState.STATE_RAISE_INCREMENT)
            time.reset()
        }
        if (g.left_bumper){
            increment_stack_count()
        }
        if(mSlideState == slideState.STATE_RAISE_INCREMENT){
            setTargetPosition(85 * clicks)
            setPower(0.0)
            newState(slideState.STATE_IDLE)
        }
        else if (mSlideState == slideState.STATE_RAISE){
            setTargetPosition(925)
            setPower(0.0)
            newState(slideState.STATE_IDLE)
        }
        else if (mSlideState == slideState.STATE_DROP){
            setTargetPosition(0)
            if (getBoundaryConditions() == slideBoundary.STATE_LOW){
                setPower(0.0)
                newState(slideState.STATE_IDLE)
            }
        }
        else if (mSlideState == slideState.STATE_IDLE){
            setPower(g.right_stick_y.toDouble())
        }
        write()
    }

    fun operate(state : slideState){
        mSlideState = state

        if (mSlideState == slideState.STATE_RAISE){
            setTargetPosition(925)
            setPower(0.0)
            newState(slideState.STATE_IDLE)
        }
        else if (mSlideState == slideState.STATE_DROP){
            setTargetPosition(0)
            if (getBoundaryConditions() == slideBoundary.STATE_LOW){
                setPower(0.0)
                newState(slideState.STATE_IDLE)
            }
        } else{
            telemetry.addData("You fucked up", getLiftHeight())
        }
        write()
    }
}