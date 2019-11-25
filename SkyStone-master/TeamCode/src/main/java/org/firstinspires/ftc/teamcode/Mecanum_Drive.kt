package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.teamcode.Universal.Math.Pose
import org.firstinspires.ftc.teamcode.Universal.Math.Vector2
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.RevBulkData
import java.util.*
import kotlin.collections.ArrayList
import kotlin.math.*

class Mecanum_Drive(hardwareMap : HardwareMap){
    var motors = ArrayList<Caching_Motor>()

    var prev_pos = ArrayList<Double>()
    var prev_heading = 0.0

    val imu : BNO055IMU
    val hub : ExpansionHubEx

    var currentWriteIndex = 0

    val TRACK_WIDTH = 15.75
    val DRIVE_WIDTH = 13.625

    var pos = Pose()

    val EPSILON = 0.001

    var currHeading = 0.0
    var headingReadCount = 0
    var headingAccessCount = 0
    val headingUpdateFrequency = 0.1

    var scale = 1.0

    var orientation : Orientation

    companion object{
        var refresh_rate = 0.5  //ngl this is kinda scary but you do what u gotta do to get 300 hz
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

        hub = hardwareMap.get(ExpansionHubEx::class.java, "Expansion Hub 2")
        imu = LynxOptimizedI2cFactory.createLynxEmbeddedImu(hub.standardModule, 0)
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)
        orientation = imu.angularOrientation
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

    fun drive(gamepad : Gamepad){
        tweakRefreshRate(gamepad)
        setPower(gamepad.left_stick_y.toDouble(), gamepad.left_stick_x.toDouble(), -gamepad.right_stick_x.toDouble())
        write()
    }

    fun f_drive(gamepad : Gamepad){
        tweakRefreshRate(gamepad)
        val r = hypot(gamepad.left_stick_y, gamepad.left_stick_x)
        var theta = atan2(gamepad.left_stick_y, gamepad.left_stick_x).toDouble()
        theta -= getExternalHeading()
        val v = Vector2(r * cos(theta), r * sin(theta))
        setPower(v, -gamepad.right_stick_x.toDouble())
        write()
    }

    fun read(data : RevBulkData) {
        motors.forEach {
            it.read(data)
        }
        headingReadCount++
        if (headingAccessCount.toDouble() / headingReadCount.toDouble() < headingUpdateFrequency) {
            headingAccessCount++
            currHeading = imu.angularOrientation.firstAngle.toDouble()
            orientation = imu.angularOrientation
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

    fun getExternalHeading() : Double{
        return orientation.firstAngle.toDouble()
    }

    fun getEstimatedPose(){
        val wheelVelocities = ArrayList<Double>()
        motors.forEachIndexed{index, motor ->
            wheelVelocities.add(motor.getCurrentPosition() - prev_pos[index])
            prev_pos[index] = motor.getCurrentPosition().toDouble()
        }
        val k = (TRACK_WIDTH + DRIVE_WIDTH) / 2.0
        val (frontLeft, rearLeft, rearRight, frontRight) = wheelVelocities

        val heading = getExternalHeading()
        val offset = Pose(wheelVelocities.sum(), rearLeft + frontRight - frontLeft - rearRight, heading - prev_heading)
        prev_heading = heading
        relativeOdometryUpdate(offset)
    }

    fun relativeOdometryUpdate(robotPoseDelta : Pose){
        val dtheta = robotPoseDelta.angle

        val (sineTerm, cosTerm) = if (abs(dtheta) < EPSILON) {
            1.0 - dtheta * dtheta / 6.0 to dtheta / 2.0
        } else {
            sin(dtheta) / dtheta to (1 - cos(dtheta)) / dtheta
        }

        val fieldPositionDelta = Vector2(sineTerm * robotPoseDelta.x - cosTerm * robotPoseDelta.y,
                cosTerm * robotPoseDelta.x + sineTerm * robotPoseDelta.y)
        fieldPositionDelta.rotate(pos.angle)

        val fieldPoseDelta = Pose(fieldPositionDelta.x, fieldPositionDelta.y, dtheta)

        pos.add(fieldPoseDelta)
    }
}