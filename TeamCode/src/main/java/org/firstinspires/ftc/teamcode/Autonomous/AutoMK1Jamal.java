package org.firstinspires.ftc.teamcode.Autonomous;/*

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ClassData.RoadRunnerData;
import org.firstinspires.ftc.teamcode.ClassData.RobotData;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;

@Disabled
@Autonomous
public class AutoMK1Jamal extends LinearOpMode {

    //Data Classes
    private RobotData robotData = new RobotData(hardwareMap);       //Basic Robot Mechanics
    private RoadRunnerData rrData = new RoadRunnerData(robotData);  //Road Runner Implementation

    @Override
    public void runOpMode() {

        //Color Selection & OpenCV
        if (robotData.getOpenCVEnabled()){
            while (opModeInInit()){
                telemetry.addLine("Select Starting Team Color");
                telemetry.update();

                //Left Side
                if (gamepad1.a){
                    robotData.setStartColor(ColorRange.BLUE);
                    break;
                }

                //Right Side
                else if (gamepad1.b){
                    robotData.setStartColor(ColorRange.RED);
                    break;
                }

                //Exception here

            }

            //OpenCV Instantiation
            ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(robotData.getStartColor())         // use a predefined color match
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                    //.setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                    .setRoi(ImageRegion.asImageCoordinates(50,50,270,150))
                    .setDrawContours(true)                        // Show contours on the Stream Preview
                    .setBlurSize(5)                               // Smooth the transitions between different colors in image
                    .build();

            robotData.setColorLocator(colorLocator);

            VisionPortal portal = new VisionPortal.Builder()
                    .addProcessor(robotData.getColorLocator())
                    .setCameraResolution(new Size(320, 240))
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .build();

        }

        //Position Selection
        while (opModeInInit()){
            //Visual Display
            if (robotData.getOpenCVEnabled()){
                if (robotData.getStartColor().equals(ColorRange.BLUE)){
                    telemetry.addLine("You selected Blue Team. Now Select a Starting Position!");
                }
                else{
                    telemetry.addLine("You selected Red Team. Now Select a Starting Position!");
                }
            }
            else{
                telemetry.addLine("OpenCV is Disabled. Now Select a Starting Position!");
            }

            telemetry.update();

            //Left Side
            if (gamepad1.dpad_left){
                robotData.setStartedLeft(true);
                rrData.setBeginPose(new Pose2d(0, 0, Math.toRadians(0)));
                break;
            }

            //Right Side
            else if (gamepad1.dpad_right){
                robotData.setStartedLeft(false);
                rrData.setBeginPose(new Pose2d(0, 0, Math.toRadians(0)));
                break;
            }
        }

        rrData.createDrive();   //Created RoadRunner Robot Object (Mechanum Drive)

        //Create Left Side Trajectory
        if (robotData.getStartedLeft()){

            //Common Positions
            Pose2d spawn = rrData.getBeginPose();

            //Each Route

            TrajectoryActionBuilder test1 = rrData.getDrive().actionBuilder(spawn)
                    .lineToX(5);

            TrajectoryActionBuilder test2 = rrData.getDrive().actionBuilder(spawn)
                    .lineToY(5);

            //Insert Trajectory Paths Here
            rrData.createTrajectoryPath(new TrajectoryActionBuilder[]{test1,test2});

            telemetry.addLine("Selected Left Side. \n Road Runner Init Done!");
            telemetry.update();
        }

        //Create Right Side Trajectory
        else{

            //Common Positions
            Pose2d spawn = rrData.getBeginPose();

            //Each Route

            TrajectoryActionBuilder test1 = rrData.getDrive().actionBuilder(spawn)
                    .lineToX(5);

            TrajectoryActionBuilder test2 = rrData.getDrive().actionBuilder(spawn)
                    .lineToY(-5);

            //Insert Trajectory Paths Here
            rrData.createTrajectoryPath(new TrajectoryActionBuilder[]{test1,test2});

            telemetry.addLine("Selected Right Side. \n Road Runner Init Done!");
            telemetry.update();
        }

        waitForStart();

        RobotData.setAutoRun(true);
        //RobotData.createRuntime();

        //Left Side Auto
        if (robotData.getStartedLeft()){
            telemetry.addLine("Beginning Auto: Left Side");
            telemetry.update();

            Actions.runBlocking(
                    new SequentialAction(
                            rrData.getTrajectory(1),
                            rrData.getTrajectory(2),
                            rrData.doReminder(0.2)
                    )
            );
        }

        //Right Side Auto
        else{
            telemetry.addLine("Beginning Auto: Right Side");
            telemetry.update();

            Actions.runBlocking(
                    new SequentialAction(
                            rrData.getTrajectory(1),
                            rrData.getTrajectory(2),
                            rrData.doReminder(1)
                    )
            );
        }
    }
}
 */