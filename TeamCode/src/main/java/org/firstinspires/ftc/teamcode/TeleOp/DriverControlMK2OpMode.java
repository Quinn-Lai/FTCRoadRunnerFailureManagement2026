package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ClassData.AprilTagVision;
import org.firstinspires.ftc.teamcode.ClassData.RoadRunnerData;
import org.firstinspires.ftc.teamcode.ClassData.RobotData;

@TeleOp
public class DriverControlMK2OpMode extends OpMode {

    //Data Classes
    //private RoadRunnerData rrData = new RoadRunnerData(robotData);  //Road Runner Implementation
    private final AprilTagVision atVision = new AprilTagVision(hardwareMap,telemetry);
    private final RobotData robotData = new RobotData(hardwareMap, telemetry, atVision);       //Basic Robot Mechanics


    //Runs Once on Init
    @Override
    public void init(){
        robotData.setMainDriver(gamepad1,"Sathya");
        atVision.initAprilTag();
    }

    @Override
    public void init_loop(){
        telemetry.addLine("Waiting for Round to Start");
        atVision.telemetryAprilTag();
        robotData.getTurret().telemetryArm(10);
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
        robotData.displayTelemetryData();

        robotData.omniDrive();

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

        robotData.checkGameTimeRumble();

        telemetry.update();
    }

    //Runs at End
    @Override
    public void stop(){
        atVision.closeVisionPortal();
    }
}