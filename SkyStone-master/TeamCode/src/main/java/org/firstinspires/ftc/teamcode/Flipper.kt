package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry

class Flipper(h : HardwareMap, telemetry: Telemetry){
    var flipper : Caching_Servo
    var clamp : Caching_Servo
    var deposit : Caching_Servo
    var turn : Caching_Servo
    var click = false
    var time = ElapsedTime()
    var t = telemetry
    var slides = Vertical_Elevator(h, telemetry)
    var clicks = 0
    var dclicks = 0

    var write_index = 0

    enum class Intake{
        wait,
        lift,
        deposit,
        end
    }

    enum class Deposit{
        deposit,
        clamp,
        unclamp
    }

    enum class Flip{
        flip1,
        realign,
        flip2,
        fullFlip
    }

    var intakeSt = Intake.end
    var depositSt = Deposit.deposit
    var flipSt = Flip.flip1

    init{
        flipper = Caching_Servo(h, "AlignerTest")
        clamp = Caching_Servo(h,"clamp")
        deposit = Caching_Servo(h,"Deposit")
        turn = Caching_Servo(h,"turn")
    }

    fun write(){
        when {
            write_index == 0 -> flipper.write()
            write_index == 1 -> clamp.write()
            write_index == 2 -> deposit.write()
            write_index == 3 -> turn.write()
        }
    }

    //These two functions are created so it is easier for autonomous programming
    fun flip(type_of_flip : Flip){
        flipSt = type_of_flip

        if(flipSt == Flip.flip1){
            flipper.setPosition(0.65)
        }else if(flipSt == Flip.fullFlip){
            flipper.setPosition(0.7)
        }else if(flipSt == Flip.flip2){
            flipper.setPosition(0.5)
        }else if(flipSt == Flip.realign){
            turn.setPosition(1.0)
        }

        write()
    }

    fun flip(Index : Int){
        if(Index == 0){
            flipper.setPosition(0.75)
        }else if(Index == 1){
            turn.setPosition(1.0)
        }else if(Index == 2){
            flipper.setPosition(0.4)
        }

        write()
    }

    fun deposit(){
        depositSt = Deposit.clamp
        when (depositSt) {
            Deposit.deposit -> {
                t.addData("State:", "Depositing Block...")

                flipper.setPosition(1.0)
                deposit.setPosition(0.0)
                //slides.setTargetPosition(3 + (clicks * 11))

                if (time.time() >= 1) {
                    depositSt = Deposit.unclamp
                    time.reset()
                }
            }
            Deposit.clamp -> {
                t.addData("State:", "Clamping Block...")

                clamp.setPosition(0.95)

                if (time.time() >= .4) {
                    depositSt = Deposit.deposit
                    time.reset()
                }
            }
            Deposit.unclamp -> {
                t.addData("State:", "Dropping Block...")

                clamp.setPosition(0.5)
            }
        }
        write()
    }

    fun initialize(){

        flipper.setPosition(1.0)
        clamp.setPosition(0.1)
        deposit.setPosition(1.0)
        turn.setPosition(0.5)
        time.startTime()
        write()
    }

    fun operate(g: Gamepad){
        if (g.a) {
            flip(Flip.flip1)
            //time.reset()
            click = true
        }else if(g.y) {
            flip(Flip.realign)
        }else if(g.x){
            flip(Flip.flip2)
        }else if(g.b){
            clicks += 1
            time.reset()
            click = true
            intakeSt = Intake.deposit
        }else{
            intakeSt = Intake.end
        }

        if(click){
            when (intakeSt) {
                Intake.wait -> {
                    t.addData("Timer:", time.time())
                    t.addData("State:", "Waiting...")
                    if (time.time() >= 2) {
                        intakeSt = Intake.lift
                        time.reset()
                    }
                }
                Intake.lift -> {
                    t.addData("State:", "Lifting")
                    deposit.setPosition(0.8)

                    if (time.time() >= 0.5) {
                        intakeSt = Intake.deposit
                        time.reset()
                    }
                }
                Intake.deposit -> {
                    t.addData("State:", "Depositing Block...")
                    flipper.setPosition(1.0)

                    if (time.time() >= 0.25) {
                        deposit()
                    }
                    if (time.time() >= 0.5) {
                        intakeSt = Intake.end
                        time.reset()
                    }
                }
                Intake.end -> {
                    t.addData("State:", "End...")
                    t.addData("Time:", time.time())

                    deposit.setPosition(0.8)
                    flipper.setPosition(1.0)
                }
            }
        }
        write()
    }

    fun ShowPos(){
        t.addData("deposit position", deposit.getPosition())
        t.addData("flipper position", flipper.getPosition())
        t.addData("clamp position", clamp.getPosition())
        t.addData("turn position", turn.getPosition())
    }

}