package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.openftc.revextensions2.RevBulkData

class Intake(hardwareMap: HardwareMap) {
    var motors : Array<Caching_Motor>

    val UPDATE_RATE = 1.0
    var write_index = 0

    init{
        motors = arrayOf(Caching_Motor(hardwareMap, "intake_left"), Caching_Motor(hardwareMap, "intake_right"))
        //motors[1].motor.direction = DcMotorSimple.Direction.REVERSE
    }

    fun read(data : RevBulkData){
        motors.map {
            it.read(data)
        }
    }

    fun write(){
        motors[write_index].write(UPDATE_RATE)
        write_index = (write_index + 1) % 2
    }

    fun setPower(power : Double){
        motors[0].setPower(power)
        motors[1].setPower(-power)
    }

    fun operate(g : Gamepad){
        setPower((g.right_trigger - g.left_trigger).toDouble())
        write()
    }
}