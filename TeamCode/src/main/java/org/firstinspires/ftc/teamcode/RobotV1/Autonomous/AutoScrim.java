package org.firstinspires.ftc.teamcode.RobotV1.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotV1.ClassData.AprilTagVision;
import org.firstinspires.ftc.teamcode.RobotV1.ClassData.RobotData;
import org.firstinspires.ftc.vision.opencv.ColorRange;

@Disabled
@Autonomous
public class AutoScrim extends LinearOpMode {

    //Data Classes
    private AprilTagVision atVision;
    private RobotData robotData;       //Basic Robot Mechanics
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

        atVision = new AprilTagVision(hardwareMap, telemetry, "blue");
        robotData = new RobotData(hardwareMap, telemetry, atVision);       //Basic Robot Mechanics
        atVision.updateAtHeight(robotData.getTurret().getHeightOfLauncher());
        atVision.initAprilTag();

        while (opModeInInit()){
            //Color Selection & OpenCV

            atVision.updateMotifCode();

            if (!(robotData.getOpenCVEnabled()) & robotData.isPendingColor()){
                robotData.selectedColor();
            }

            else if (robotData.isPendingColor() && robotData.getOpenCVEnabled()){
                telemetry.addLine("Select Starting Team Color");
                telemetry.update();

                //Left Side
                if (gamepad1.a){
                    RobotData.setStartColor(ColorRange.BLUE);
                    robotData.selectedColor();

                    //openCVData.createColorLocator(RobotData.getStartColor());
                    //openCVData.createVisionPortal();
                }

                //Right Side
                else if (gamepad1.b) {
                    RobotData.setStartColor(ColorRange.RED);
                    robotData.selectedColor();

                    //openCVData.createColorLocator(RobotData.getStartColor());
                    //openCVData.createVisionPortal();
                }
            }

            else if (!robotData.isPendingColor() && robotData.isPendingPosition()){
                //Visual Display
                if (robotData.getOpenCVEnabled()){
                    if (RobotData.getStartColor().equals(ColorRange.BLUE)){
                        telemetry.addLine("You selected Blue Team. Now Select a Starting Position!");
                    }
                    else{
                        telemetry.addLine("You selected Red Team. Now Select a Starting Position!");
                    }
                }

                else{
                    telemetry.addLine("OpenCV is Disabled. Select a Starting Position!");
                }

                telemetry.update();

                //Left Side
                if (gamepad1.dpad_left){
                    RobotData.setStartedLeft(true);
                    robotData.selectedPosition();
                }

                //Right Side
                else if (gamepad1.dpad_right){
                    RobotData.setStartedLeft(false);
                    robotData.selectedPosition();
                }
            }

            else{
                telemetry.addLine("Completed Initialization.... \n --------------------------" );
                telemetry.addData("Chosen Color",RobotData.getStartColor());
                telemetry.addData("Chosen Position",RobotData.getStartingPosition());
                telemetry.addData("Found Motif", atVision.foundMotif());
                if (atVision.foundMotif()){
                    telemetry.addData("Motif Code", atVision.getMotifCode()[0] + ", " + atVision.getMotifCode()[1] + ", " +atVision.getMotifCode()[2]);
                }

                atVision.telemetryAprilTag();
                telemetry.update();
            }
        }

        waitForStart();

        //RobotData.setAutoRun(true);
        RobotData.createRuntime();

        robotData.getCarosel().updatePattern(atVision.getMotifCode());
        robotData.getCarosel().setInventoryAuto();

        //Left Side Auto
        if (RobotData.getStartedLeft()){

            if (atVision.foundMotif()){

                telemetry.addLine("Beginning Auto: Left Side");
                telemetry.update();

                robotData.getTurret().quickTurn(-1);
                sleep(670);
                robotData.getTurret().killSpinnerServo();

                robotData.getTurret().aimBall(2.51);
                robotData.getCarosel().powerIntakeServo();

                robotData.getCarosel().cyclePattern(0);
                telemetry.update();

                sleep(8000);
                robotData.getCarosel().ariseElevator();
                telemetry.update();
                sleep(2000);
                robotData.getTurret().angleRobot(1.7);
                robotData.getCarosel().deriseElevatorPrimative();
                sleep(2000);

                robotData.getTurret().aimBall(2.51);
                robotData.getCarosel().cyclePattern(1);

                sleep(3000);
                robotData.getCarosel().ariseElevator();
                telemetry.update();
                sleep(2000);
                robotData.getTurret().angleRobot(1.7);
                robotData.getCarosel().deriseElevatorPrimative();
                sleep(2000);

                robotData.getTurret().aimBall(2.51);
                robotData.getCarosel().cyclePattern(2);

                sleep(3000);
                robotData.getCarosel().ariseElevator();
                telemetry.update();
                sleep(2000);
                robotData.getTurret().angleRobot(1.7);
                robotData.getCarosel().deriseElevatorPrimative();
            }

            else{
                telemetry.addLine("Didn't Find Motif Oof");
                telemetry.update();
            }

        }

        //Right Side Auto
        else{

            if (atVision.foundMotif()){

                telemetry.addLine("Beginning Auto: Right Side");
                telemetry.update();

                robotData.getTurret().quickTurn(1);
                sleep(670);
                robotData.getTurret().killSpinnerServo();

                robotData.getTurret().aimBall(2.51);
                robotData.getCarosel().powerIntakeServo();

                robotData.getCarosel().cyclePattern(0);
                telemetry.update();

                sleep(8000);
                robotData.getCarosel().ariseElevator();
                telemetry.update();
                sleep(2000);
                robotData.getTurret().angleRobot(1.7);
                robotData.getCarosel().deriseElevatorPrimative();
                sleep(2000);

                robotData.getTurret().aimBall(2.51);
                robotData.getCarosel().cyclePattern(1);

                sleep(3000);
                robotData.getCarosel().ariseElevator();
                telemetry.update();
                sleep(2000);
                robotData.getTurret().angleRobot(1.7);
                robotData.getCarosel().deriseElevatorPrimative();
                sleep(2000);

                robotData.getTurret().aimBall(2.51);
                robotData.getCarosel().cyclePattern(2);

                sleep(3000);
                robotData.getCarosel().ariseElevator();
                telemetry.update();
                sleep(2000);
                robotData.getTurret().angleRobot(1.7);
                robotData.getCarosel().deriseElevatorPrimative();
            }

            else{
                telemetry.addLine("Didn't Find Motif Oof");
                telemetry.update();
            }
        }

        LFmotor.setPower(0.4);
        RFmotor.setPower(0.6);
        LBmotor.setPower(0.4);
        RBmotor.setPower(0.6);

        sleep(800);

        LFmotor.setPower(0);
        RFmotor.setPower(0);
        LBmotor.setPower(0);
        RBmotor.setPower(0);

    }

//    @Override
//    public void start(){
//
//    }
//
//    @Override
//    public void loop(){
//
//    }
//
//    @Override
//    public void stop() {
//
//        telemetry.addLine("Autonomous Completed!");
//        telemetry.addData("Time Spent: ",RobotData.getRuntime());
//
//        if (robotData.getOpenCVEnabled()){
//            //openCVData.closePortal();
//        }
//
//        atVision.closeVisionPortal();
//    }
}