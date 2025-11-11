package org.firstinspires.ftc.teamcode.RobotV1.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Config
public class testCRServo extends LinearOpMode {

    private CRServo testServo1;
    private CRServo testServo2;

    private double speed = 0.5;

    @Override
    public void runOpMode(){

        testServo1 = hardwareMap.get(CRServo.class,"testServo1");
        testServo2 = hardwareMap.get(CRServo.class,"testServo2");

        waitForStart();

        while (opModeIsActive()){


            if(gamepad1.square){
                testServo1.setPower(speed);
                testServo2.setPower(-speed);
            }

            else if (gamepad1.triangle){
                sleep(100);
                speed += 0.1;
            }

            else if (gamepad1.cross){
                sleep(100);
                speed -= 0.1;
            }

            else{
                testServo1.setPower(0);
                testServo2.setPower(0);
            }

            telemetry.addData("Speed",speed);
            telemetry.update();

        }

    }

}
