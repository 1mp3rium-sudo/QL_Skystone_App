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
    var doubleClick = ElapsedTime()


    enum class flip_state{
        STATE_CLAMP,
        STATE_FLIP,
        STATE_DEPOSIT,
        CASE_RIGHT,
        CASE_LEFT,
        STATE_DROP,
        STATE_IDLE,
        STATE_DELAY
    }

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

    fun start(){
        clamp.setPosition(0.8)
        deposit.setPosition(0.0)
        turn.setPosition(0.5)
        time.startTime()
        doubleClick.startTime()
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

    fun operate(g1: Gamepad, g2 : Gamepad){
        if(g2.y) {
            newState(flip_state.STATE_DELAY)
        }
        if(g2.a){
            newState(flip_state.STATE_DEPOSIT)
        }
        if(g1.left_bumper){
            newState(flip_state.STATE_DROP)
        }
        if(g2.dpad_right){
            //newState(flip_state.CASE_RIGHT)
        }
        if(g2.dpad_left){
            //newState(flip_state.CASE_LEFT)
        }

        if (betterFlipState == flip_state.STATE_FLIP){
            clamp.setPosition(.55)
            if(time.time() >= 0.3){
                flipper.setPosition(0.55)
            }
            if(time.time() >= 1.2){
                betterFlipState = flip_state.STATE_CLAMP
            }
        }
        else if (betterFlipState == flip_state.STATE_CLAMP){
            clamp.setPosition(0.95)
            if (time.time() >= 2) {
                newState(flip_state.STATE_IDLE)
            }
        }else if(betterFlipState == flip_state.STATE_IDLE){
            flipper.setPosition(0.95)
            deposit.setPosition(0.0)
            clamp.setPosition(0.8)
        }else if(betterFlipState == flip_state.STATE_DEPOSIT){
            deposit.setPosition(0.85)
        }else if(betterFlipState == flip_state.STATE_DROP) {
            clamp.setPosition(0.55)
            if (time.time() >= 2.0) {
                newState(flip_state.STATE_IDLE)
            }
        }else if(betterFlipState == flip_state.STATE_DELAY){
            if(time.time() >= 0.5){
                newState(flip_state.STATE_FLIP)
            }
        }else if(betterFlipState == flip_state.CASE_LEFT){
            clamp.setPosition(.55)
            if(time.time() >= 0.3){
                flipper.setPosition(0.3)
                time.reset()
                //flipper.setPosition(.45)
            }
            if(time.time() >= .3){
                turn.setPosition(1.0)
                time.reset()
            }
            if(time.time() >= .2){
                flipper.setPosition(.15)
            }
            if(time.time() >= 1.2){
                betterFlipState = flip_state.STATE_CLAMP
            }

            turn.setPosition(0.0)
        }else if(betterFlipState == flip_state.CASE_RIGHT){
            clamp.setPosition(.55)
            if(time.time() >= 0.3){
                flipper.setPosition(0.3)
                time.reset()
                //flipper.setPosition(.45)
            }
            if(time.time() >= .3){
                turn.setPosition(1.0)
                time.reset()
            }
            if(time.time() >= .2){
                flipper.setPosition(.15)
            }
            if(time.time() >= 1.2){
                betterFlipState = flip_state.STATE_CLAMP
            }

            turn.setPosition(0.0)
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