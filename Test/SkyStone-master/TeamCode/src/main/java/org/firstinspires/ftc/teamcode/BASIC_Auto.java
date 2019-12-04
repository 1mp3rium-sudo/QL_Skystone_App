package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous(name="Concept: VuMark Id Webcam", group ="Concept")
public class BASIC_Auto extends OpMode {
    Caching_Motor[] motors;
    ObjectLocalization tf = new ObjectLocalization();

    public void init(){
        motors[0] = new Caching_Motor(hardwareMap, "up_left");
        motors[1] = new Caching_Motor(hardwareMap, "back_left");
        motors[2] = new Caching_Motor(hardwareMap, "back_right");
        motors[3] = new Caching_Motor(hardwareMap, "up_right");

        tf.start();
    }

    public void loop(){
        tf.LocObject();


    }
}
