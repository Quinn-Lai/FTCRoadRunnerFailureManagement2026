package org.firstinspires.ftc.teamcode.RobotV2.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotDataV2;

@Config
@TeleOp

public class TunePIDCarosel extends LinearOpMode {

    public static int ticks = 64;
    private DcMotorEx testMotor;

    public static double kF = 0;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;


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

        testMotor = (DcMotorEx)(hardwareMap.get(DcMotor.class,"caroselMotor"));

        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.square){
                testMotor.setTargetPosition(ticks);
                testMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                testMotor.setVelocity(PIDShooter(testMotor.getCurrentPosition(), ticks));
            }

            telemetry.addData("PID Co: ", getPIDCos());
            telemetry.addData("PID kF: ", kF);
            telemetry.addData("Current Pos: ", testMotor.getCurrentPosition());
            telemetry.addData("True TPS: ", testMotor.getVelocity());
            telemetry.addData("Ticks Desired: ", ticks);
            telemetry.addData("Tick Error:", testMotor.getCurrentPosition() - ticks);

            telemetry.update();

        }

    }

}

