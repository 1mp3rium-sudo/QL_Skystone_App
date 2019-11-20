package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap

class Flipper(hardwareMap: HardwareMap) {
    var servos = arrayOf(Caching_Servo(hardwareMap, "Deposit"), Caching_Servo(hardwareMap, "clamp"))
    var write_index = 0

    fun clamp(){
        servos[1].setPosition(0.95)
    }

    fun unclamp(){
        servos[1].setPosition(0.5)
    }

    fun flip_out(){
        servos[0].setPosition(0.0)
    }

    fun flip_in(){
        servos[0].setPosition(1.0)
    }

    fun write(){
        servos[write_index].write()
        write_index = (write_index + 1) % 2
    }

    fun operate(g : Gamepad){
        if (g.left_bumper){
            clamp()
        }
        else if (g.right_bumper){
            unclamp()
        }

        if (g.dpad_up){
            flip_out()
        }
        else if (g.dpad_down){
            flip_in()
        }

        write()
    }
}