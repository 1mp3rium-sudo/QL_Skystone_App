package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.openftc.revextensions2.RevBulkData

class Intake(hardwareMap: HardwareMap) {
    var motors : Array<Caching_Motor>

    val UPDATE_RATE = 1.0
    var write_index = 0
    val open : Array<Caching_Servo>
    var time = ElapsedTime()

    init{
        motors = arrayOf(Caching_Motor(hardwareMap, "intake_left"), Caching_Motor(hardwareMap, "intake_right"))
        open = arrayOf(Caching_Servo(hardwareMap, "intake_left_jaw"), Caching_Servo(hardwareMap, "intake_right_jaw"))
        close()
        time.startTime()
    }

    enum class clamp{
        OPEN,
        CLOSE
    }

    var clampst = clamp.CLOSE
    fun read(data : RevBulkData){
        motors.map {
            it.read(data)
        }
    }

    fun write(){
        motors[write_index].write()
        open[write_index].write()
        write_index = (write_index + 1) % 2
    }

    fun setPower(power : Double){
        motors[0].setPower(power)
        motors[1].setPower(-power)
    }

    fun open(){
        open[0].setPosition(0.4)
        open[1].setPosition(0.6)
    }

    fun close(){
        open[0].setPosition(0.7) //0.7
        open[1].setPosition(0.2) //0.2
    }

    fun newState(clampState: clamp){
        clampst = clampState
        time.reset()
    }

    fun operate(g1 : Gamepad, g2: Gamepad){
        setPower(0.25 * (g1.right_trigger - g1.left_trigger).toDouble())

        if(g2.y){
            newState(clamp.OPEN)
        }

        if(clampst == clamp.OPEN){
            open()
            if (time.time() >= 3){
                close()
            }
        }

        write()
    }
}