/*
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotV1.ClassData.RoadRunnerData;
import org.firstinspires.ftc.teamcode.RobotV1.ClassData.RobotData;

@TeleOp
@Disabled
public class DriverControlMK1Jamal extends LinearOpMode {

    //Data Classes
    private RobotData robotData = new RobotData(hardwareMap);       //Basic Robot Mechanics
    private RoadRunnerData rrData = new RoadRunnerData(robotData);  //Road Runner Implementation

    @Override
    public void runOpMode(){

        waitForStart();

        RobotData.createRuntime();
        robotData.setMainDriver(gamepad1,"Sathya");
        telemetry.addLine("Beginning TeleOp: Main Driver set to Gamepad1");

        while(opModeIsActive()){

            //Data
            telemetry.addData("Time: ", RobotData.getRuntime());
            telemetry.addData("Main Driver: ", robotData.getMainDriver());
            telemetry.update();

            robotData.omniDrive();

            //Drivers
            if (gamepad1.share){
                robotData.setMainDriver(gamepad1,"Sathya");
            }
            else if (gamepad2.share){
                robotData.setMainDriver(gamepad2,"Will");
            }
        }
    }
}

*/