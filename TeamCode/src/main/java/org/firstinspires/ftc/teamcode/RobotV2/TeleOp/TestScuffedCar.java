package org.firstinspires.ftc.teamcode.RobotV2.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class TestScuffedCar extends LinearOpMode {

    private DcMotor LFmotor;
    private DcMotor LBmotor;
    private DcMotor RFmotor;
    private DcMotor RBmotor;

    @Override
    public void runOpMode(){

        LFmotor  = hardwareMap.get(DcMotor.class, "LFmotor");
        RFmotor = hardwareMap.get(DcMotor.class, "RFmotor");
        LBmotor = hardwareMap.get(DcMotor.class, "LBmotor");
        RBmotor = hardwareMap.get(DcMotor.class, "RBmotor");

        LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RFmotor.setDirection(DcMotor.Direction.FORWARD);
        RBmotor.setDirection(DcMotor.Direction.FORWARD);
        LFmotor.setDirection(DcMotor.Direction.REVERSE);
        LBmotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            omniDrive(gamepad1);
        }

    }

    public void omniDrive(Gamepad gamepad) {

        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double z = gamepad.right_stick_x;

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

}
