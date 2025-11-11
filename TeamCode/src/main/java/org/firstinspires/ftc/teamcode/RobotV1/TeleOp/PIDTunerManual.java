package org.firstinspires.ftc.teamcode.RobotV1.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class PIDTunerManual extends LinearOpMode {

    private double TPS = 1000;
    private DcMotorEx testMotor;

    private double kP = 0;

    private double kI = 0;
    private double kD = 0;


    private ElapsedTime PIDTimer = new ElapsedTime();

    private double integralSum = 0;
    private double lastError = 0;

    public String getPIDCos(){
        return kP + ", " + kI + ", " + kD;
    }

    public void changeKp(double val){
        kP += val;
    }

    public void changeKI(double val){
        kI += val;
    }

    public void changeKD(double val){
        kD += val;
    }

    public double PIDShooter(double current, double desired) {

        double currentTime = PIDTimer.seconds();

        double output = 0;

        double error = desired - current;

        integralSum += error * currentTime;

        double derivate = (error - lastError) / currentTime;

        lastError = error;

        PIDTimer.reset();

        output = (error * kP) + (derivate * kD) + (integralSum * kI);

        return output;
    }

    @Override
    public void runOpMode(){

        testMotor = (DcMotorEx)(hardwareMap.get(DcMotor.class,"testMotor"));
        testMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.left_bumper){
                testMotor.setVelocity(TPS);
            }
            else {
                testMotor.setVelocity(0);
            }

            if (gamepad1.dpad_up){
                TPS += 10;
            }

            else if (gamepad1.dpad_down){
                TPS -= 10;
            }

            else if (gamepad1.dpad_left){
                TPS -= 5;
            }

            else if (gamepad1.dpad_right){
                TPS += 5;
            }

            if (gamepad1.square){
                changeKp(10);
            }

            else if (gamepad1.circle){
                changeKp(-10);
            }

            else if (gamepad1.triangle){
                changeKD(10);
            }

            else if (gamepad1.cross){
                changeKD(-10);
            }

            else if (gamepad2.left_bumper){
                changeKI(-10);
            }

            else if (gamepad2.right_bumper){
                changeKI(10);
            }

            telemetry.addData("PID Co: ", getPIDCos());
            telemetry.addData("True TPS: ", testMotor.getVelocity());
            telemetry.addData("Ticks Per Second: ", TPS);
            telemetry.addData("TPS Error:", testMotor.getVelocity() - TPS);

        }

    }

}
