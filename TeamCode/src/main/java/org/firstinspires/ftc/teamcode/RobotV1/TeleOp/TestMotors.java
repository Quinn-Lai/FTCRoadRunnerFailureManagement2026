package org.firstinspires.ftc.teamcode.RobotV1.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotV1.ClassData.AprilTagVision;
import org.firstinspires.ftc.teamcode.RobotV1.ClassData.RobotData;

@TeleOp
public class TestMotors extends LinearOpMode {

    private AprilTagVision atData = new AprilTagVision(hardwareMap,telemetry, "blue");
    private RobotData robot = new RobotData(hardwareMap,telemetry,atData);

    private DcMotorEx leftSpinnerMotor;
    private DcMotorEx rightSpinnerMotor;

    //Encoders 28
    private double TPR = 28.0;
    private double RPM = 6000;
    private double maxSpeed = RPM * TPR /60;
    private int speed = 1500;

    public void updateSpeed(){
        if (speed <= (maxSpeed-100)) {
            sleep(100);
            speed += 100;
        }
    }

    public void decSpeed(){
        if (speed >= 100) {
            sleep(100);
            speed -= 100;
        }
    }


    @Override
    public void runOpMode(){

        atData.updateAtHeight(robot.getTurret().getHeightOfLauncher());

        robot.getDriveTrain().setMode(gamepad1,"Sathya");
        atData.initAprilTag();

        leftSpinnerMotor = (DcMotorEx)(hardwareMap.get(DcMotor.class,"leftSpinnerMotor"));
        rightSpinnerMotor = (DcMotorEx)(hardwareMap.get(DcMotor.class,"rightSpinnerMotor"));

        leftSpinnerMotor.setDirection(DcMotor.Direction.FORWARD);
        rightSpinnerMotor.setDirection(DcMotor.Direction.REVERSE); //Guess

        leftSpinnerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightSpinnerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftSpinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSpinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

            telemetry.addData("Speed",leftSpinnerMotor.getVelocity());
            telemetry.update();

        }


    }

}
