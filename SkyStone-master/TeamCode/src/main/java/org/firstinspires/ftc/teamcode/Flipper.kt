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
    var time = ElapsedTime()
    var t = telemetry
    var slides = Vertical_Elevator(h, telemetry)
    var clicks = 0
    var dclicks = 0
    var click = false


    enum class flip_state{
        STATE_CLAMP,
        STATE_FLIP,
        STATE_IDLE
    }


    enum class Intake{
        wait,
        lift,
        manual,
        deposit,
        end
    }

    enum class Deposit{
        deposit,
        clamp,
        unclamp,
        reset
    }

    enum class Flip{
        flip1,
        realign,
        flip2,
        fullFlip,
        reset
    }

    var intakeSt = Intake.end
    var depositSt = Deposit.deposit
    var flipSt = Flip.flip1

    var betterFlipState = flip_state.STATE_IDLE

    init{
        flipper = Caching_Servo(h, "AlignerTest")
        clamp = Caching_Servo(h,"clamp")
        deposit = Caching_Servo(h,"Deposit")
        turn = Caching_Servo(h,"turn")
    }

    fun write(){
        flipper.write()
        clamp.write()
        deposit.write()
        turn.write()
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
            flipper.setPosition(0.65)
        }else if(Index == 1){
            //turn.setPosition(1.0)
        }else if(Index == 2){
            flipper.setPosition(0.5)
        }

        write()
    }

    fun deposit(){
        if(time.time() >= 2.0){
            depositSt = Deposit.clamp
        }
        if(depositSt == Deposit.deposit){
            t.addData("State:", "Depositing Block...")

            flipper.setPosition(1.0)
            deposit.setPosition(0.0)
            //slides.setTargetPosition(3 + (clicks * 11))

            time.reset()
            if (time.time() >= 1) {
                depositSt = Deposit.unclamp
                time.reset()
            }
        }
        if(depositSt == Deposit.clamp){
            t.addData("State:", "Clamping Block...")

            clamp.setPosition(0.95)

            if (time.time() >= 0.4) {
                depositSt = Deposit.deposit
                time.reset()
            }
        }
        if(depositSt == Deposit.unclamp){
            t.addData("State:", "Dropping Block...")

            clamp.setPosition(0.55)
            if (time.time() >= .4) {
                depositSt = Deposit.deposit
                time.reset()
            }
        }
        write()
    }

    fun start(){
        flipper.setPosition(1.0)
        clamp.setPosition(0.55)
        deposit.setPosition(0.0)
        turn.setPosition(0.5)
        time.startTime()
        write()
    }

    fun initialize(){
        flipper.setPosition(0.3)
        clamp.setPosition(0.7)
        deposit.setPosition(0.0)
        turn.setPosition(0.5)
        write()
    }

    fun newState(flipState: flip_state){
        betterFlipState = flipState
        time.reset()
    }

    fun operate(g: Gamepad){
        if (g.dpad_up && time.time() >= 1) {
            flip(dclicks)
            dclicks % 3
            dclicks += 1
            time.reset()
        } else if(g.y) {
            newState(flip_state.STATE_FLIP)
        }else if(g.b && time.time() >= 1){
            clicks += 1
            time.reset()
            deposit()
        }else if(g.a){
            start()
        }

        if (betterFlipState == flip_state.STATE_FLIP){
            flipper.setPosition(0.5)
            if(time.time() >= 1.0){
                t.addData("Hooray", "YAY")
                newState(flip_state.STATE_CLAMP)
            }
        }
        else if (betterFlipState == flip_state.STATE_CLAMP){
            clamp.setPosition(0.95)
            if (time.time() >= 1.0) {
                newState(flip_state.STATE_IDLE)
            }
        }else if(betterFlipState == flip_state.STATE_IDLE){
            flipper.setPosition(1.0)
        }

        t.addData("time", time.time())
        write()
    }

    fun ShowPos(){
        t.addData("deposit position", deposit.getPosition())
        t.addData("flipper position", flipper.getPosition())
        t.addData("clamp position", clamp.getPosition())
        t.addData("turn position", turn.getPosition())
    }

}