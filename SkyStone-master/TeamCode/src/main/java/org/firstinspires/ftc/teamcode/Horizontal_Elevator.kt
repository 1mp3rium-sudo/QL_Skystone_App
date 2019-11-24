package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import java.util.*
import kotlin.math.atan2

class Horizontal_Elevator(hardwareMap: HardwareMap) {
    var motor = Caching_CR_Servo(hardwareMap, "horizontal_1")

    val SLIDE_REACH = 8.0

    val offset = Vector2d(4.0, 4.0)
    var foundation_angle = 0.0
    var stack_count = 0

    val MAX_STONES = 12
    val MAX_SAFE_1x2_COUNT = 6

    fun setPower(power : Double){
        motor.setPower(power)
    }

    fun setPower(power : Float){
        motor.setPower(power.toDouble())
    }

    fun write(){
        motor.write()
    }

    fun operate(g : Gamepad){
        setPower(g.left_stick_x)
        write()
    }

    fun getDepositLocation(pos : Pose2d, center_pos : Pose2d) : Pair<Pose3d>{
        val target_pos = Pose3d(getDepositPoint(pos, center_pos), getDepositHeight().toDouble())
        var target_index = when{
            getDoubleStackPositions(center_pos, stack_count)[0].epsilonEquals(target_pos.pos) -> 0
            else -> 1
        }
        
        var target_angle = 0
       
        return Pair(target_pos, target_angle)
    }

    fun getDepositPoint(pos : Pose2d, center_pos: Pose2d) : Pose2d{
        val target : Vector2d = when{
            stack_count < MAX_STONES - MAX_SAFE_1x2_COUNT -> getBestDoubleStackPosition(center_pos, pos, stack_count)
            else -> getSingleStackPosition(center_pos)
        }

        return getTargetDepositPoint(pos, target)
    }
    
    fun getBestDoubleStackPosition(center_pos : Pose2d, pos : Pose2d, stack_count : Int){
        val possibilities = getDoubleStackPositions(center_pos, stack_count)
        
        return when{
            getTargetDepositPoint(pos, possibilities[0]).vec().distTo(pos.vec()) > getTargetDepositPoint(pos, possibilities[1]).vec().distTo(pos.vec()) -> possibilities[1]
            getTargetDepositPoint(pos, possibilities[0]).vec().distTo(pos.vec()) <= getTargetDepositPoint(pos, possibilities[1]).vec().distTo(pos.vec()) -> possibilities[0]
            else -> pos
        }
    }

    fun getDepositHeight() : Int{
        return when{
            stack_count > MAX_STONES - MAX_SAFE_1x2_COUNT -> 4 * (((MAX_STONES - MAX_SAFE_1x2_COUNT) / 2) + (stack_count - (MAX_STONES - MAX_SAFE_1x2_COUNT)))
            stack_count <= MAX_STONES - MAX_SAFE_1x2_COUNT -> 4 * (MAX_STONES - MAX_SAFE_1x2_COUNT) / 2
            else -> -1
        }
    }

    fun increment_stack_count(){
        stack_count++
    }

    fun decrement_stack_count(){
        stack_count--
    }

    fun getDoubleStackPositions(center_pos : Pose2d, stack_count : Int) : Array<Vector2d>{
        return when{
            stack_count % 4 == 0 -> arrayOf(center_pos.vec() + offset.rotated(foundation_angle), reflectVectorOverAngle(offset.rotated(foundation_angle), foundation_angle))
            stack_count % 4 == 1 -> arrayOf(center_pos.vec() + offset.rotated(Math.PI + foundation_angle), reflectVectorOverAngle(offset.rotated(Math.PI + foundation_angle), foundation_angle))
            stack_count % 4 == 2 -> arrayOf(center_pos.vec() + offset.rotated((Math.PI / 2) + foundation_angle), reflectVectorOverAngle(offset.rotated((Math.PI / 2) + foundation_angle), foundation_angle))
            stack_count % 4 == 3 -> arrayOf(center_pos.vec() + offset.rotated(((3 * Math.PI) / 2) + foundation_angle), reflectVectorOverAngle(offset.rotated(((3 * Math.PI) / 2) + foundation_angle), foundation_angle))
            else -> center_pos.vec()
        }
    }
                                            
    private fun reflectVectorOverVector(target : Vector2d, base : Vector2d){
        return new Vector2d(-target.rotated(-base.angle()).x, target.rotated(-base.angle()).y).rotated(base.angle())
    }
                                                                                                       
    private fun reflectVectorOverAngle(target : Vector2d, base : Double){
        return new Vector2d(-target.rotated(-base).x, target.rotated(-base).y).rotated(base)
    }

    fun getSingleStackPosition(center_pos : Pose2d) : Vector2d{
        return center_pos.vec() + offset.rotated(foundation_angle)
    }

    fun getTargetDepositPoint(pos : Pose2d, targetStack : Vector2d) : Pose2d{
        val targets = getCircleLineIntersectionPoint(Point(pos.x, pos.y), Point(targetStack.x, targetStack.y), SLIDE_REACH).map{
            it.toVector()
        }

        return when{
            targets[0].distTo(pos.vec()) > targets[1].distTo(pos.vec()) -> Pose2d(targets[1], atan2(pos.y - targets[1].y, pos.x - targets[1].x))
            targets[0].distTo(pos.vec()) < targets[1].distTo(pos.vec()) -> Pose2d(targets[0], atan2(pos.y - targets[0].y, pos.x - targets[0].x))
            targets[0].distTo(pos.vec()) == targets[1].distTo(pos.vec()) -> Pose2d(targets[1], atan2(pos.y - targets[1].y, pos.x - targets[1].x))
            else -> pos
        }
    }

    private fun getCircleLineIntersectionPoint(pointA: Point,  center: Point, radius: Double): List<Point> {
        val baX = center.x - pointA.x
        val baY = center.y - pointA.y
        val caX = center.x - pointA.x
        val caY = center.y - pointA.y

        val a = baX * baX + baY * baY
        val bBy2 = baX * caX + baY * caY
        val c = caX * caX + caY * caY - radius * radius

        val pBy2 = bBy2 / a
        val q = c / a

        val disc = pBy2 * pBy2 - q
        if (disc < 0) {
            return Collections.emptyList()
        }
        // if disc == 0 ... dealt with later
        val tmpSqrt = Math.sqrt(disc)
        val abScalingFactor1 = -pBy2 + tmpSqrt
        val abScalingFactor2 = -pBy2 - tmpSqrt

        val p1 = Point(pointA.x - baX * abScalingFactor1, pointA.y - baY * abScalingFactor1)
        if (disc == 0.0) { // abScalingFactor1 == abScalingFactor2
            return Collections.singletonList(p1)
        }
        val p2 = Point(pointA.x - baX * abScalingFactor2, pointA.y - baY * abScalingFactor2)
        return Arrays.asList(p1, p2)
    }

    internal class Point(var x: Double, var y: Double) {

        override fun toString(): String {
            return "Point [x=$x, y=$y]"
        }

        fun toVector() : Vector2d{
            return Vector2d(x, y)
        }
    }
}