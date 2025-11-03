package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ClassData.AprilTagVision;
import org.firstinspires.ftc.teamcode.ClassData.RoadRunnerData;
import org.firstinspires.ftc.teamcode.ClassData.RobotData;

@TeleOp
public class DriverControlBlue extends OpMode {

    //Data Classes
    private RoadRunnerData rrData;  //Road Runner Implementation
    private AprilTagVision atVision; //April Tag Vision
    private RobotData robotData;       //Basic Robot Mechanics

    //Runs Once on Init
    @Override
    public void init(){

        atVision = new AprilTagVision(hardwareMap,telemetry,"blue");
        robotData = new RobotData(hardwareMap, telemetry, atVision);       //Basic Robot Mechanics
        atVision.updateAtHeight(robotData.getTurret().getHeightOfLauncher());

        rrData = new RoadRunnerData(robotData);

        robotData.getDriveTrain().setMode(gamepad1,"Auto Aim Mode");
        atVision.initAprilTag();
    }

    @Override
    public void init_loop(){
        telemetry.addLine("Waiting for Round to Start");
        atVision.telemetryAprilTag();

        if (atVision.canSeeAT()){
            robotData.updateTelemetry(telemetry);
            telemetry.addData("Yaw: ", atVision.getYaw());
            telemetry.addData("X: ", atVision.getXPos());
            robotData.getTurret().telemetryArm(atVision.getDisp());
        }

        telemetry.update();
    }

    //Runs Once on Start
    @Override
    public void start(){
        RobotData.createRuntime();
        //rrData.createDashboard();
    }

    //Loops after Start
    @Override
    public void loop(){

        robotData.updateTelemetry(telemetry);
        robotData.getDriveTrain().displayTelemetryData();

        //-----------------------------------------------------

        //Mode Switching
        if (gamepad1.shareWasPressed() || gamepad2.shareWasPressed()){
            robotData.getDriveTrain().switchMode();
        }

        if (robotData.getDriveTrain().getCurrentMode()){
            robotData.getDriveTrain().setMode(gamepad1,"Auto Aim Mode");
        }
        else{
            robotData.getDriveTrain().setMode(gamepad1,"Manual Mode");
        }

        robotData.getDriveTrain().updateModeColor();
        robotData.getDriveTrain().omniDrive();

        //-----------------------------------------------------

        //Auto Aim Mode

        if (robotData.getDriveTrain().getCurrentMode()){

            //Toggle Motors & Aim
            if (gamepad1.circleWasPressed()){
                robotData.getTurret().switchTurretMode();
            }

            //Passive Auto Aim
            if (robotData.getTurret().isToggleTurretAim()){
                if (atVision.canSeeAT()){
                    robotData.getTurret().aimBall(atVision.getDisp());
                    robotData.updateTelemetry(telemetry);
                    robotData.getTurret().telemetryArm(atVision.getDisp());
                }
            }
            else{
                robotData.getTurret().killShooterPower();
            }

            //Auto Center
            if (gamepad1.dpad_right){
                robotData.getTurret().centerShot(atVision.getYaw(),true);
            }
            else{
                robotData.getTurret().killSpinnerServo();
            }

            //Auto Sort & Inventory
            robotData.getCarosel().updateCaroselReaction();

            //Subzone when it detects something
            while (robotData.getCarosel().getArtifactDetected()){
                robotData.getDriveTrain().setMode(gamepad1,"Auto Aim Mode");
                robotData.getDriveTrain().omniDrive();

                robotData.getCarosel().checkCycleEnd();
            }
        }


        //-----------------------------------------------------

        //Manual Mode

        else{

            //Cycle Carosel

            if (gamepad1.squareWasPressed()){
                robotData.getCarosel().startCycling(robotData.getCarosel().incrementCurrentPos(1));
            }
            robotData.getCarosel().checkCycleEnd();

            if (gamepad1.optionsWasPressed()){
                robotData.getCarosel().incCaroselMotor();
            }

            if (gamepad1.dpad_left){
                robotData.getTurret().angleLeftSpin();
            }
            else if (gamepad1.dpad_right) {
                robotData.getTurret().angleRightSpin();
            }
            else{
                robotData.getTurret().killSpinnerServo();
            }

        }

        //-----------------------------------------------------

        //Shared Controls

        //Elevator
        if (gamepad1.rightBumperWasPressed()){
            robotData.getCarosel().ariseElevator();
        }
        robotData.getCarosel().checkDeriseElevator();

        //Intake
        if (gamepad1.crossWasPressed()){
            robotData.getCarosel().intakeArtifact();
        }

        //-----------------------------------------------------

        //Auto Park

        if (robotData.getDriveTrain().getMode().left_trigger > 0.75 && robotData.getDriveTrain().getMode().right_trigger > 0.75){
            rrData.addTeleOpAction(rrData.getTestSeqAction()); //Replace with Auto Park
            rrData.runTeleOpActions();
        }

        //-----------------------------------------------------

        //Telemetry

        //End Game Rumbling
        robotData.getDriveTrain().checkEndgame();

        //Check If Can See April Tag
        if (atVision.canSeeAT()){
            telemetry.addLine("Eyes Open");
        }
        else{
            telemetry.addLine("Eyes Closed");
        }

        //Nerd Stuff
        telemetry.addData("Yaw", atVision.getYaw());
        telemetry.addData("Color Detected", robotData.getCarosel().getColorIntake());
        telemetry.addData("X: ", atVision.getXPos());

        telemetry.addData("RGB", robotData.getCarosel().getCS().red() + "," + robotData.getCarosel().getCS().green() + "," + robotData.getCarosel().getCS().blue());
        telemetry.addData("HSV",  robotData.getCarosel().getHSV()[0] + "," + robotData.getCarosel().getHSV()[1] + "," + robotData.getCarosel().getHSV()[2]);

        robotData.getCarosel().printInv();
        telemetry.addData("Pos: ", robotData.getCarosel().getCurrentCycle());

        telemetry.update();
    }

    //Runs at End
    @Override
    public void stop(){
        atVision.closeVisionPortal();
    }
}