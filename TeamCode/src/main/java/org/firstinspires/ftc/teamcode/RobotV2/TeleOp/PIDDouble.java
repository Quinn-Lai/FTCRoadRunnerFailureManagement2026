package org.firstinspires.ftc.teamcode.RobotV2.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotConstantsV2;

@TeleOp
@Config
public class                                                                                                                                                                                                                              PIDDouble extends LinearOpMode {

    public static double TPS = 1000;
    private DcMotorEx motor1;
    private DcMotorEx motor2;

    private ElapsedTime PIDTimer = new ElapsedTime();

    private double integralSum = 0;
    private double lastError = 0;

    public String getPIDCos(){
        return String.format("kP: %s, kI: %s, kD: %s, kF: %s ", RobotConstantsV2.kP, RobotConstantsV2.kI, RobotConstantsV2.kD, RobotConstantsV2.kF);
    }

    public double PIDShooter(double current, double desired) {

        double currentTime = PIDTimer.seconds();

        double output = 0;

        double error = desired - current;

        integralSum += error * currentTime;

        double derivate = (error - lastError) / currentTime;

        lastError = error;

        PIDTimer.reset();

        output = (error * RobotConstantsV2.kP) + (derivate * RobotConstantsV2.kD) + (integralSum * RobotConstantsV2.kI) + (desired * RobotConstantsV2.kF);

        return output;
    }

    @Override
    public void runOpMode(){

        motor1 = (DcMotorEx)(hardwareMap.get(DcMotor.class,"motor1"));
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor2 = (DcMotorEx)(hardwareMap.get(DcMotor.class,"motor2"));
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.left_bumper){
                motor1.setPower(PIDShooter(motor1.getVelocity(),TPS));
                motor2.setPower(PIDShooter(motor1.getVelocity(),TPS));
            }
            else if(gamepad1.right_bumper){
                motor1.setVelocity(TPS);
                motor2.setVelocity(TPS);
            }

            else {
                motor1.setVelocity(0);
                motor2.setVelocity(0);
            }

            telemetry.addData("PID ADJUSTED 1: ", PIDShooter(motor1.getVelocity(),TPS));
            telemetry.addData("PID ADJUSTED 2: ", PIDShooter(motor2.getVelocity(),TPS));
            telemetry.addData("PID Co: ", getPIDCos());
            telemetry.addData("True TPS 1: ", motor1.getVelocity());
            telemetry.addData("True TPS 2: ", motor2.getVelocity());
            telemetry.addData("Ticks Per Second: ", TPS);
            telemetry.addData("TPS Error:", motor1.getVelocity() - TPS);
            telemetry.addData("TPS Error:", motor2.getVelocity() - TPS);

            telemetry.update();
        }

    }

}