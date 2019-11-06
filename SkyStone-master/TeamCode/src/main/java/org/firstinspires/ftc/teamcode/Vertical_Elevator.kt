package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Universal.Motion.MotionProfile
import org.openftc.revextensions2.RevBulkData
import kotlin.math.abs

class Vertical_Elevator(map : HardwareMap, t : Telemetry){
    var motors : Array<Caching_Motor>

    val UPDATE_RATE = 1
    var write_index = 0

    var telemetry = t

    enum class slideState{
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
        motors[1].motor.direction = DcMotorSimple.Direction.REVERSE
        //motors[0].motor.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    fun read(data : RevBulkData){
        motors.map {
            it.read(data)
        }
    }

    fun write(){
        motors[write_index].write()
        write_index = (write_index + 1) % 2
    }

    fun setPower(power : Double){
        motors.map{
            it.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
        when getBoundaryConditions(){
            slideBoundary.STATE_OPTIMAL ->
                motors.map{ it.setPower(power) }
            slideBoundary.STATE_HIGH ->
                motors.map { it.setPower(abs(power)) }
            slideBoundary.STATE_LOW ->
                motors.map { it.setPower(-abs(power)) }
        }
    }
    
    fun getLiftHeight() : Double{
        return ((motors[0].getCurrentPosition() + motors[1].getCurrentPosition()) / 2).toDouble()
    }
    
    fun getBoundaryConditions() : slideBoundary{
        if (getLiftHeight() < -25 && getLiftHeight() > -925){
            return slideBoundary.STATE_OPTIMAL
        }
        else if (getLiftHeight() < -925){
            return slideBoundary.STATE_HIGH
        }
        else if (getLiftHeight() > -25){
            return slideBoundary.STATE_LOW
        }
        else{
            return slideBoundary.STATE_UNKNOWN
        }
    }

    fun newState(s : slideState){
        mSlideState = s
    }

    fun setTargetPosition(target : Int){
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
        if (g.y){
            newState(slideState.STATE_RAISE)
        }
        else if (g.a){
            newState(slideState.STATE_DROP)
        }
        else if (g.b){
            newState(slideState.STATE_DROP)
        }

        if (mSlideState == slideState.STATE_RAISE){
            setTargetPosition(-925)
            if (abs(getLiftHeight() + 925) < 25){
                setPower(0.0)
                newState(slideState.STATE_IDLE)
            }
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
        else{
            telemetry.addData("You fucked up", getLiftHeight())
        }
        write()
    }
}
