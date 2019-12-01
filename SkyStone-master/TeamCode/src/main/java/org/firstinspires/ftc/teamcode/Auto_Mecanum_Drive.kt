package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.teamcode.Universal.Math.Pose
import org.firstinspires.ftc.teamcode.Universal.Math.Vector2
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.RevBulkData
import java.util.*
import kotlin.collections.ArrayList
import kotlin.math.*

class Auto_Mecanum_Drive(hardwareMap : HardwareMap){
    var motors = ArrayList<Caching_Motor>()

    var prev_pos = ArrayList<Double>()
    var prev_heading = 0.0

    var currentWriteIndex = 0

    val TRACK_WIDTH = 15.75
    val DRIVE_WIDTH = 13.625

    var pos = Pose()

    val EPSILON = 0.001

    var time = ElapsedTime()

    var scale = 1.0

    var fine_tune = 1.0

    var previous = false
    var slow_mode = false

    companion object{
        var refresh_rate = 0.5  //ngl this is kinda scary but you do what u gotta do to get 300 hz
        var kPa = 0.01
    }

    init{
        motors.add(Caching_Motor(hardwareMap, "up_left"))
        motors.add(Caching_Motor(hardwareMap, "back_left"))
        motors.add(Caching_Motor(hardwareMap, "back_right"))
        motors.add(Caching_Motor(hardwareMap, "up_right"))

        motors.forEach {
            it.motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        motors[0].motor.direction = DcMotorSimple.Direction.REVERSE
        motors[1].motor.direction = DcMotorSimple.Direction.REVERSE

        time.startTime()
    }

    private fun tweakRefreshRate(gamepad : Gamepad){
        if (gamepad.dpad_up){
            raiseRefreshRate()
        }
        else if (gamepad.dpad_down){
            dropRefreshRate()
        }
    }

    private fun raiseRefreshRate(){
        var num = (1 / refresh_rate).toInt()
        num++
        refresh_rate = 1.div(num.toDouble())
    }

    private fun dropRefreshRate(){
        var num = (1 / refresh_rate).toInt()
        if (num > 1) {
            num--
            refresh_rate = 1.div(num.toDouble())
        }
    }

    fun getRefreshRate() : Double{
        return refresh_rate
    }

    fun getPower() : Array<Double>{
        return arrayOf(motors[0].prev_power, motors[1].prev_power, motors[2].prev_power, motors[3].prev_power)
    }

    fun turnTo(heading : Double) : Boolean{
        val error = 0.0//heading - getExternalHeading()
        val power = kPa * error
        if (abs(error) > Math.toRadians(10.0)) {
            setPower(0.0, 0.0, power)
            return true
        }
        else{
            setPower(0.0, 0.0, 0.0)
            return false
        }
    }

    fun pivotTo(heading : Double, fPower : Double) : Boolean{
        val error = 0.0//heading - getExternalHeading()
        val power = kPa * error * 0.5
        if (abs(error) > Math.toRadians(10.0)) {
            setPower(fPower, 0.0, power)
            return true
        }
        else{
            setPower(0.0, 0.0, 0.0)
            return false
        }
    }

    fun drive(gamepad : Gamepad){
        //tweakRefreshRate(gamepad)
        if (isPress(gamepad.right_bumper)){
            slow_mode = !slow_mode
        }
        previous = gamepad.right_bumper

        if (slow_mode){
            fine_tune = 0.5
        }
        else{
            fine_tune = 1.0
        }
        setPower(fine_tune * gamepad.left_stick_y.toDouble(), fine_tune * gamepad.left_stick_x.toDouble(), -0.5 * gamepad.right_stick_x.toDouble())
        write()
    }

    fun isPress(test : Boolean) : Boolean{
        return test && !previous
    }

    fun angleWrap(angle : Double) : Double{
        return (angle + (2 * Math.PI)) % (2 * Math.PI)
    }

    /*fun f_drive(gamepad1 : Gamepad){
        //tweakRefreshRate(gamepad1)
        if (isPress(gamepad1.right_bumper)){
            slow_mode = !slow_mode
        }
        previous = gamepad1.right_bumper

        if (slow_mode){
            fine_tune = 0.5
        }
        else{
            fine_tune = 1.0
        }

        val r = hypot(gamepad1.left_stick_y, gamepad1.left_stick_x)
        val theta = atan2(/*-*/gamepad1.left_stick_y, gamepad1.left_stick_x).toDouble()
        val v = Vector2(r * cos(theta), r * sin(theta))
        v.rotate(angleWrap(getExternalHeading()))
        setPower(v, -0.5 * gamepad1.right_stick_x)
        write()
    }*/

    fun read(data : RevBulkData) {
        motors.forEach {
            it.read(data)
        }
    }

    fun setPower(y : Double, x : Double, rightX : Double){

        var FrontLeftVal = y - x + rightX
        var FrontRightVal = y + x - rightX
        var BackLeftVal = y + x + rightX
        var BackRightVal = y - x - rightX

        //Move range to between 0 and +1, if not already
        val wheelPowers = doubleArrayOf(FrontRightVal, FrontLeftVal, BackLeftVal, BackRightVal)
        Arrays.sort(wheelPowers)
        if (wheelPowers[3] > 1) {
            FrontLeftVal /= wheelPowers[3]
            FrontRightVal /= wheelPowers[3]
            BackLeftVal /= wheelPowers[3]
            BackRightVal /= wheelPowers[3]
        }
        motors[0].setPower(FrontLeftVal)
        motors[1].setPower(BackLeftVal)
        motors[2].setPower(BackRightVal)
        motors[3].setPower(FrontRightVal)
    }

    fun setPower(v : Vector2, rightX : Double){
        setPower(v.y, v.x, rightX)
    }

    fun write(){
        motors[currentWriteIndex].write()
        currentWriteIndex = (currentWriteIndex + 1) % 4
    }
}