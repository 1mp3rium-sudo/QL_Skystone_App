package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.openftc.revextensions2.RevBulkData
import java.util.*

class Flipper(h : HardwareMap, telemetry: Telemetry){
    var flipper : Caching_Servo
    var clamp : Caching_Servo
    var deposit : Caching_Servo
    var turn : Caching_Servo
    var leftpm : Caching_Servo
    var rightpm : Caching_Servo
    var time = ElapsedTime()
    var t = telemetry
    var grabbed = false
    var turnPos = 0.5
    var sensorDistance: DistanceSensor
    var dist = 0.0
    var rcase: Int = 0
    var previous = false

    companion object {
        const val case_right_turn_value = 0.775
        const val case_left_turn_value = 0.15
        const val case_center_turn_value = 0.5

        const val handshake_flip_position = 0.45 //THIS IS GOING BACKWARDS 1 -> 0

        const val turnPos_IDOL = 0.4635
        const val flipperPos_IDOL = 0.945 //THIS IS GOING BACKWARDS 1 -> 0
        const val DepositPos_IDOL = 0.0 //THIS IS GOING BACKWARDS 1 -> 0

        const val DepositPos = 0.82
        const val Deposit_Clearance_DROPPING_Block = 0.85
        const val Deposit_Clearance_HANDSHAKE = .06

        const val Flipper_Midway_REALLIGN = 0.75 //THIS IS GOING BACKWARDS 1 -> 0
    }

    private fun clamp(){
        clamp.setPosition(0.95)
    }

    private fun unclamp(){
        clamp.setPosition(.5)
    }

    enum class flip_state{
        STATE_CLAMP,
        STATE_FLIP,
        STATE_DEPOSIT,
        STATE_REALLIGN,
        STATE_DROP,
        STATE_IDLE
    }

    var betterFlipState = flip_state.STATE_IDLE

    init{
        flipper = Caching_Servo(h, "AlignerTest")
        clamp = Caching_Servo(h,"clamp")
        deposit = Caching_Servo(h,"Deposit")
        turn = Caching_Servo(h,"turn")
        rightpm = Caching_Servo(h, "rightpm")
        leftpm = Caching_Servo(h, "leftpm")
        sensorDistance = h.get(DistanceSensor::class.java, "cds")
    }

    fun write(){
        flipper.write()
        clamp.write()
        deposit.write()
        turn.write()
        leftpm.write()
        rightpm.write()
    }

    fun start(){
        deposit.setPosition(0.025)
        turn.setPosition(turnPos_IDOL)
        clamp()
        time.startTime()
        flipper.setPosition(flipperPos_IDOL)
        write()
    }

    fun initialize(){
        flipper.setPosition(0.3)
        clamp.setPosition(0.7)
        deposit.setPosition(0.0)
        turn.setPosition(turnPos)
        leftpm.setPosition(0.2)
        rightpm.setPosition(0.75)
        write()
    }

    fun read(){
        dist = sensorDistance.getDistance(DistanceUnit.CM)
    }

    fun newState(flipState: flip_state){
        betterFlipState = flipState
        time.reset()

    }

    private fun getCase() : Int{
        read()
        if (dist >= 6.5 && dist <= 9.25) {
            //Case regular
            //6.75 - 7.5
            rcase = 0
        } else if (dist >= 5.45 && dist <= 6.45) {
            //Case left
            //5.45 - 6
            rcase = 1
        } else if (dist >= 9.5 && dist <= 12.0) {
            //Case right
            //9.5 - 10.5
            rcase = 2
        }
        return rcase
    }

    fun grabPlatform(){
        leftpm.setPosition(0.95)
        rightpm.setPosition(0.0)
        grabbed = true
    }

    fun resetPlatform(){
        leftpm.setPosition(0.2)
        rightpm.setPosition(0.75)
        grabbed = false
    }

    private fun isPress(clicked : Boolean) : Boolean{
        return clicked && !previous
    }



    fun operate(g1: Gamepad, g2 : Gamepad){
        if(g2.right_bumper){
            newState(flip_state.STATE_DEPOSIT)
        }
        if(g1.left_bumper){
            newState(flip_state.STATE_DROP)
        }
        if(g2.b){
            newState(flip_state.STATE_IDLE)
        }
        if(isPress(g1.a)){
            if(grabbed){
                resetPlatform()
                grabbed = false
            }else{
                grabPlatform()
                grabbed = true
            }
        }
        previous = g1.a

        if(g2.left_bumper){
            if(getCase() == 0) {
                //Case regular
                turnPos = case_center_turn_value
                newState(flip_state.STATE_FLIP)
            }
            if(getCase() == 2){
                //Case Right
                turnPos = case_right_turn_value
                newState(flip_state.STATE_REALLIGN)
            }
            if(getCase() == 1){
                //Case Left
                turnPos = case_left_turn_value
                newState(flip_state.STATE_REALLIGN)
            }
        }
        if(g2.right_trigger >= 0.5){
            turnPos = case_right_turn_value
            newState(flip_state.STATE_REALLIGN)
        }
        if(g2.left_trigger >= 0.5){
            turnPos = case_left_turn_value
            newState(flip_state.STATE_REALLIGN)
        }

        if (betterFlipState == flip_state.STATE_FLIP){
            unclamp()
            if(time.time() >= 0.5){ //Wait for setup procedure before flipping
                turn.setPosition(turnPos)
                flipper.setPosition(handshake_flip_position)
                if(time.time() >= 1.3){
                    newState(flip_state.STATE_CLAMP)
                }
            }
        }
        else if (betterFlipState == flip_state.STATE_CLAMP){
            clamp()
            if (time.time() >= 1.0) {   //Wait for 1 second for the flipper to go back
                newState(flip_state.STATE_IDLE)
            }
        }else if(betterFlipState == flip_state.STATE_IDLE){
            turnPos = turnPos_IDOL
            flipper.setPosition(flipperPos_IDOL)
            turn.setPosition(turnPos)
            deposit.setPosition(DepositPos_IDOL)
            clamp()
        }else if(betterFlipState == flip_state.STATE_DEPOSIT){
            deposit.setPosition(DepositPos)
        }else if(betterFlipState == flip_state.STATE_DROP) {
            unclamp()
            if(time.time() >= 0.3){
                deposit.setPosition(Deposit_Clearance_DROPPING_Block)
            }
        }else if(betterFlipState == flip_state.STATE_REALLIGN){
            unclamp()
            if(time.time() >= 0.5){ //0.5
                flipper.setPosition(Flipper_Midway_REALLIGN)
            }
            if(time.time() >= 1.0){ //0.6
                turn.setPosition(((turnPos-0.5)/2) + 0.5)
                //flipper.setPosition(handshake_flip_position)
            }
            if(time.time() >= 1.8){
                turn.setPosition(turnPos)
                flipper.setPosition(handshake_flip_position)
            }
            if(time.time() >= 2.5){
                clamp()
            }
            if(time.time() >= 3.0){
                deposit.setPosition(Deposit_Clearance_HANDSHAKE)
                flipper.setPosition(0.5)
            }
            if(time.time() >= 3.5){
                flipper.setPosition(0.7)
            }

            if(time.time() >= 3.5){
                newState(flip_state.STATE_IDLE)
            }
        }

        t.addData("time", time.time())
        write()
    }

    fun operate(sequece : Int){
        if(sequece == 0){
            newState(flip_state.STATE_DEPOSIT)
        }
        if(sequece == 1){
            newState(flip_state.STATE_DROP)
        }
        if(sequece == 2){
            newState(flip_state.STATE_IDLE)
        }

        if(sequece == 3){
            if(getCase() == 0) {
                //Case regular
                turnPos = case_center_turn_value
                newState(flip_state.STATE_FLIP)
            }
            if(getCase() == 2){
                //Case Right
                turnPos = case_right_turn_value
                newState(flip_state.STATE_REALLIGN)
            }
            if(getCase() == 1){
                //Case Left
                turnPos = case_left_turn_value
                newState(flip_state.STATE_REALLIGN)
            }
        }
        if (betterFlipState == flip_state.STATE_FLIP){
            unclamp()
            if(time.time() >= 0.5){ //Wait for setup procedure before flipping
                turn.setPosition(turnPos)
                flipper.setPosition(handshake_flip_position)
                if(time.time() >= 1.3){
                    newState(flip_state.STATE_CLAMP)
                }
            }
        }
        else if (betterFlipState == flip_state.STATE_CLAMP){
            clamp()
            if (time.time() >= 1.0) {   //Wait for 1 second for the flipper to go back
                newState(flip_state.STATE_IDLE)
            }
        }else if(betterFlipState == flip_state.STATE_IDLE){
            turnPos = turnPos_IDOL
            flipper.setPosition(flipperPos_IDOL)
            turn.setPosition(turnPos)
            deposit.setPosition(DepositPos_IDOL)
            clamp()
        }else if(betterFlipState == flip_state.STATE_DEPOSIT){
            deposit.setPosition(DepositPos)
        }else if(betterFlipState == flip_state.STATE_DROP) {
            unclamp()
            if(time.time() >= 0.3){
                deposit.setPosition(Deposit_Clearance_DROPPING_Block)
            }
        }else if(betterFlipState == flip_state.STATE_REALLIGN){
            unclamp()
            if(time.time() >= 0.5){ //0.5
                flipper.setPosition(Flipper_Midway_REALLIGN)
            }
            if(time.time() >= 1.0){ //0.6
                turn.setPosition(((turnPos-0.5)/2) + 0.5)
                //flipper.setPosition(handshake_flip_position)
            }
            if(time.time() >= 1.8){
                turn.setPosition(turnPos)
                flipper.setPosition(handshake_flip_position)
            }
            if(time.time() >= 2.5){
                clamp()
            }
            if(time.time() >= 3.0){
                deposit.setPosition(Deposit_Clearance_HANDSHAKE)
                flipper.setPosition(0.5)
            }
            if(time.time() >= 3.5){
                flipper.setPosition(0.7)
            }

            if(time.time() >= 3.5){
                newState(flip_state.STATE_IDLE)
            }
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