package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import kotlin._Assertions;

@TeleOp
public class TestMotors extends LinearOpMode {

    private DcMotorEx leftSpinnerMotor;
    private DcMotorEx rightSpinnerMotor;

    private int speed = 1500;

    public void updateSpeed(){
        if (speed <= 2600) {
            speed += 100;
        }
    }

    public void decSpeed(){
        if (speed >= 100) {
            speed -= 100;
        }
    }


    @Override
    public void runOpMode(){

        leftSpinnerMotor = (DcMotorEx) hardwareMap.get(DcMotor.class,"leftSpinnerMotor");
        rightSpinnerMotor = (DcMotorEx) hardwareMap.get(DcMotor.class,"rightSpinnerMotor");

        leftSpinnerMotor.setDirection(DcMotor.Direction.FORWARD);
        rightSpinnerMotor.setDirection(DcMotor.Direction.REVERSE); //Guess

        leftSpinnerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightSpinnerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()){

            if(gamepad1.square){
                leftSpinnerMotor.setDirection(DcMotor.Direction.FORWARD);
                rightSpinnerMotor.setDirection(DcMotor.Direction.REVERSE);

                leftSpinnerMotor.setVelocity(speed);
                rightSpinnerMotor.setVelocity(speed);
            }

            else if (gamepad1.circle){
                leftSpinnerMotor.setDirection(DcMotor.Direction.REVERSE);
                rightSpinnerMotor.setDirection(DcMotor.Direction.FORWARD);

                leftSpinnerMotor.setVelocity(speed);
                rightSpinnerMotor.setVelocity(speed);
            }

            else if (gamepad1.triangle){
                updateSpeed();
            }

            else if (gamepad1.cross){
                decSpeed();
            }

            else{
                leftSpinnerMotor.setVelocity(0);
                rightSpinnerMotor.setVelocity(0);
            }

            telemetry.addData("Speed",speed);
            telemetry.update();

        }


    }

}
