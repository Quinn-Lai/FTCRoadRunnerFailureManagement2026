package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Config
public class PIDTuner extends LinearOpMode {

    public static double TPS = 1000;
    private DcMotorEx testMotor;

    public static double kP = 0;

    public static double kI = 0;
    public static double kD = 0;

    private ElapsedTime PIDTimer = new ElapsedTime();

    private double integralSum = 0;
    private double lastError = 0;

    public String getPIDCos(){
        return kP + ", " + kI + ", " + kD;
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

        telemetry.addData("Error P:",error);
        telemetry.addData("Derivate: ", derivate);
        telemetry.addData("IntegralSum", integralSum);

        return output;
    }

    @Override
    public void runOpMode(){

        testMotor = (DcMotorEx)(hardwareMap.get(DcMotor.class,"testMotor"));
        testMotor.setDirection(DcMotor.Direction.REVERSE);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.left_bumper){
                testMotor.setVelocity(PIDShooter(testMotor.getVelocity(),TPS));
            }
            else {
                testMotor.setVelocity(0);
            }

            telemetry.addData("PID ADJUSTED: ", PIDShooter(testMotor.getVelocity(),TPS));
            telemetry.addData("PID Co: ", getPIDCos());
            telemetry.addData("True TPS: ", testMotor.getVelocity());
            telemetry.addData("Ticks Per Second: ", TPS);
            telemetry.addData("TPS Error:", testMotor.getVelocity() - TPS);


            telemetry.update();
        }

    }

}
