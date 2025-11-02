package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class testServoPos extends LinearOpMode {

    public static double position = 0;
    private Servo testServo;

    @Override
    public void runOpMode(){

        testServo = hardwareMap.get(Servo.class,"testServo");

        waitForStart();

        while (opModeIsActive()){

            testServo.setPosition(position);

        }

    }


}
