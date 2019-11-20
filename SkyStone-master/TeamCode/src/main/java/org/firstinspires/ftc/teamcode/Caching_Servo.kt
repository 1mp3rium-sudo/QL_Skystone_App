package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.abs

class Caching_Servo(hardwareMap: HardwareMap, name : String){
    var servo = hardwareMap.get(Servo::class.java, name)
    var prev_pos = -2.0
    var query = 0.0

    val EPSILON = 0.001

    fun setPosition(pos : Double){
        if (abs(pos - prev_pos) > EPSILON){
            query = pos
        }
    }

    fun getPosition() : Double{
        return prev_pos
    }

    fun write(){
        servo.position = query
        prev_pos = query
    }
}