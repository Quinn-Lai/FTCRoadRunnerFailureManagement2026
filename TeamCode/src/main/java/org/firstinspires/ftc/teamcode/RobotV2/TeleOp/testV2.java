package org.firstinspires.ftc.teamcode.RobotV2.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotV1.ClassData.RobotData;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotDataV2;

@Disabled
@TeleOp
public class testV2 extends LinearOpMode {

    private DcMotor LFmotor;
    private DcMotor RFmotor;
    private DcMotor LBmotor;
    private DcMotor RBmotor;
    private Servo transferServo;
    private DcMotor shooterMotor;
    private DcMotor intakeMotor;

    @Override
    public void runOpMode(){

        transferServo = hardwareMap.get(Servo.class,"transferServo");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotor.class,"intakeMotor");

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

            if (gamepad1.dpad_up){
                transferServo.setPosition(0.6);
            }

            else if (gamepad1.dpad_down){
                transferServo.setPosition(0.9);
            }

            if (gamepad1.circle){
                shooterMotor.setPower(-1);
            }

//            else{
//                shooterMotor.setPower(0);
//            }

            if (gamepad1.square){
                intakeMotor.setPower(-.67);
            }

            if (gamepad1.triangle){
                intakeMotor.setPower(0);
                shooterMotor.setPower(0);
            }

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

