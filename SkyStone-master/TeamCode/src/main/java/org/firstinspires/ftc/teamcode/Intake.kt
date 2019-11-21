package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.openftc.revextensions2.RevBulkData

class Intake(hardwareMap: HardwareMap) {
    var motors : Array<Caching_Motor> = arrayOf(Caching_Motor(hardwareMap, "intake_left"), Caching_Motor(hardwareMap, "intake_right"))
    var servos : Array<Caching_Servo> = arrayOf(Caching_Servo(hardwareMap, "intake_left_jaw"), Caching_Servo(hardwareMap, "intake_right_jaw"))

    val UPDATE_RATE = 1.0
    var write_index = 0
    var servo_write_index = 0

    init{
        close()
    }

    fun read(data : RevBulkData){
        motors.map {
            it.read(data)
        }
    }

    fun write(){
        motors[write_index].write(UPDATE_RATE)
        write_index = (write_index + 1) % 2
        servos[servo_write_index].write()
        servo_write_index = (servo_write_index + 1) % 2
    }

    fun close(){
        servos[0].setPosition(0.7)
        servos[1].setPosition(0.2)
    }

    fun open(){
        servos[0].setPosition(0.5)
        servos[1].setPosition(0.5)
    }

    fun setPower(power : Double){
        motors[0].setPower(0.3 * power)
        motors[1].setPower(-0.3 * power)
    }

    fun operate(g : Gamepad){
        setPower((g.right_trigger - g.left_trigger).toDouble())
        if (g.left_bumper){
            open()
        }
        else if (g.right_bumper){
            close()
        }
        write()
    }
}