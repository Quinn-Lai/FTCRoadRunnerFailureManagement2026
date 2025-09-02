package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TestDrive extends LinearOpMode {

    private DcMotor LFmotor;
    private DcMotor RFmotor;
    private DcMotor LBmotor;
    private DcMotor RBmotor;
    private boolean robotCentric;
    private IMU imu;

    @Override
    public void runOpMode(){

        LFmotor  = hardwareMap.get(DcMotor.class, "LFmotor");
        RFmotor = hardwareMap.get(DcMotor.class, "RFmotor");
        LBmotor = hardwareMap.get(DcMotor.class, "LBmotor");
        RBmotor = hardwareMap.get(DcMotor.class, "RBmotor");

        //imu = hardwareMap.get(IMU.class, "imu");
        robotCentric = true;

        RFmotor.setDirection(DcMotor.Direction.FORWARD);
        RBmotor.setDirection(DcMotor.Direction.FORWARD);
        LFmotor.setDirection(DcMotor.Direction.REVERSE);
        LBmotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            omniDrive();

            if (gamepad1.share || gamepad2.share){
                if (robotCentric){
                    robotCentric = false;
                }
                else{
                    robotCentric = true;
                }
            }
        }

    }

    public void omniDrive(){

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double z = gamepad1.right_stick_x;

        //Robot POV
        if (robotCentric){
            x *= 1.1;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(z), 1);
            double frontLeftPower = (y + x + z) / denominator;
            double backLeftPower = (y - x + z) / denominator;
            double frontRightPower = (y - x - z) / denominator;
            double backRightPower = (y + x - z) / denominator;

            LFmotor.setPower(frontLeftPower);
            LBmotor.setPower(backLeftPower);
            RFmotor.setPower(frontRightPower);
            RBmotor.setPower(backRightPower);
        }

        //Field POV
        else{
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(z), 1);
            double frontLeftPower = (rotY + rotX + z) / denominator;
            double backLeftPower = (rotY - rotX + z) / denominator;
            double frontRightPower = (rotY - rotX - z) / denominator;
            double backRightPower = (rotY + rotX - z) / denominator;

            LFmotor.setPower(frontLeftPower);
            LBmotor.setPower(backLeftPower);
            RFmotor.setPower(frontRightPower);
            RBmotor.setPower(backRightPower);
        }
    }

}
