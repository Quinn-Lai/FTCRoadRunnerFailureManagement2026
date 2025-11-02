package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.ClassData.AprilTagVision;
import org.firstinspires.ftc.teamcode.ClassData.RoadRunnerData;
import org.firstinspires.ftc.teamcode.ClassData.RobotConstants;
import org.firstinspires.ftc.teamcode.ClassData.RobotData;

@TeleOp
public class DriverControlMK2OpMode extends OpMode {

    //Data Classes
    //private RoadRunnerData rrData = new RoadRunnerData(robotData);  //Road Runner Implementation
    private AprilTagVision atVision;
    private RobotData robotData;       //Basic Robot Mechanics
    private RobotConstants constants;


    //Runs Once on Init
    @Override
    public void init(){

        constants = new RobotConstants();
        atVision = new AprilTagVision(hardwareMap,telemetry);
        robotData = new RobotData(hardwareMap, telemetry, atVision);       //Basic Robot Mechanics
        atVision.updateAtHeight(robotData.getTurret().getHeightOfLauncher());

        robotData.getDriveTrain().setMainDriver(gamepad1,"Sathya");
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

        robotData.getDriveTrain().setMainDriver(gamepad1,"Sathya");
        robotData.updateTelemetry(telemetry);

        robotData.getDriveTrain().displayTelemetryData();
        robotData.getDriveTrain().omniDrive();

        /*
        //Drivers
        if (gamepad1.share){
            robotData.setMainDriver(gamepad1,"Sathya");
        }
        else if (gamepad2.share){
            robotData.setMainDriver(gamepad2,"Will");
        }
        */

        /*
        if (gamepad2.a){
            rrData.addTeleOpAction(rrData.getTestSeqAction());
            rrData.runTeleOpActions();
        }
         */

        robotData.getDriveTrain().checkGameTimeRumble();

        if (atVision.canSeeAT()){
            telemetry.addLine("Eyes Open");
        }


        if (gamepad1.share) {
            if (atVision.canSeeAT()){
                robotData.getTurret().aimBall(atVision.getDisp());
                robotData.updateTelemetry(telemetry);
                robotData.getTurret().telemetryArm(atVision.getDisp());
            }
            else{
                telemetry.addLine("-------------------- \n \nOpen Yo Eyes");
            }

        }

        else{
            robotData.getTurret().killShooterPower();
        }

        if (gamepad1.dpad_right){
            robotData.getTurret().centerShot(atVision.getYaw(),true);
        }

        else{
            robotData.getTurret().killSpinnerServo();
        }

        if (gamepad1.triangleWasPressed()){
            robotData.getCarosel().cyclePositionalSpot(robotData.getCarosel().incrementCurrentPos(1), gamepad1);
        }

        else if (gamepad1.triangleWasReleased()){
            robotData.getCarosel().killCaroselPower();
        }

        //robotData.getCarosel().checkIntakeArtifact(gamepad1);

        if (gamepad1.squareWasPressed()){
            robotData.getCarosel().ariseElevator();
        }

        robotData.getCarosel().checkDeriseElevator();

        if (gamepad1.crossWasPressed()){ //intake
            robotData.getCarosel().intakeArtifact();
        }

//        if (gamepad2.dpad_left) {
//            RobotConstants.xDrag -= 0.1;
//        }
//
//        else if (gamepad2.dpad_right){
//            RobotConstants.xDrag += 0.1;
//        }
//
//        if (gamepad2.dpad_up){
//            RobotConstants.yDrag -= 0.1;
//        }
//
//        else if (gamepad2.dpad_down){
//            RobotConstants.yDrag += 0.1;
//        }

        telemetry.addData("Yaw", atVision.getYaw());
        telemetry.addData("Color Detected", robotData.getCarosel().getColorIntake());
        telemetry.addData("X: ", atVision.getXPos());

        telemetry.addData("Color", robotData.getCarosel().getCS().red() + "," + robotData.getCarosel().getCS().green() + "," + robotData.getCarosel().getCS().blue());
        robotData.getCarosel().printInv();
        telemetry.addData("Pos: ", robotData.getCarosel().getCurrentCaroselPos());

        telemetry.update();
    }

    //Runs at End
    @Override
    public void stop(){
        atVision.closeVisionPortal();
    }
}