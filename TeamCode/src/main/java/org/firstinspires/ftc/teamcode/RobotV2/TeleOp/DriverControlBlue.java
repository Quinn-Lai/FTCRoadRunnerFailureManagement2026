package org.firstinspires.ftc.teamcode.RobotV2.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotV2.ClassData.AprilTagVisionV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RoadRunnerDataV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotDataV2;

@Disabled
@TeleOp
public class DriverControlBlue extends OpMode {

    //Data Classes
    private RoadRunnerDataV2 rrData;  //Road Runner Implementation
    private AprilTagVisionV2 atVision; //April Tag Vision
    private RobotDataV2 robotData;       //Basic Robot Mechanics

    //Runs Once on Init
    @Override
    public void init(){

        atVision = new AprilTagVisionV2(hardwareMap,telemetry,"blue");
        robotData = new RobotDataV2(hardwareMap, telemetry, atVision);       //Basic Robot Mechanics
        atVision.updateAtHeight(robotData.getTurret().getHeightOfLauncher());

        rrData = new RoadRunnerDataV2(robotData);

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
        RobotDataV2.createRuntime();
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

            }
            else{
                robotData.getTurret().killShooterPower();
            }

        }


        //-----------------------------------------------------

        //Manual Mode

        else{


            if (gamepad1.optionsWasPressed()){
                robotData.getTurret().switchTurretMode();
            }

            else if (gamepad1.circleWasPressed()){
                robotData.getTurret().switchTurretManualClose();
            }

            if (robotData.getTurret().isToggleTurretAim()){
                robotData.getTurret().aimBall(3);
            }

            else if (robotData.getTurret().isToggleTurretManualClose()){
                robotData.getTurret().aimBall(1.7);
            }

            else{
                robotData.getTurret().killShooterPower();
            }

        }

        //-----------------------------------------------------

        //Shared Controls


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



        telemetry.update();
    }

    //Runs at End
    @Override
    public void stop(){
        atVision.closeVisionPortal();
    }
}