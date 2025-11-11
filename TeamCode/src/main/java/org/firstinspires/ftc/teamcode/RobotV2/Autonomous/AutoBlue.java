package org.firstinspires.ftc.teamcode.RobotV2.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotV2.ClassData.AprilTagVisionV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RoadRunnerDataV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotDataV2;
import org.firstinspires.ftc.vision.opencv.ColorRange;

@Autonomous
public class AutoBlue extends OpMode {

    //Data Classes
    private AprilTagVisionV2 atVision;
    //private OpenCVData openCVData;
    private RobotDataV2 robotData;       //Basic Robot Mechanics
    private RoadRunnerDataV2 rrData;  //Road Runner Implementation

    @Override
    public void init(){

        atVision = new AprilTagVisionV2(hardwareMap, telemetry, "blue");
        //openCVData = new OpenCVData(hardwareMap);
        robotData = new RobotDataV2(hardwareMap, telemetry, atVision);       //Basic Robot Mechanics
        rrData = new RoadRunnerDataV2(robotData);  //Road Runner Implementation
        atVision.updateAtHeight(robotData.getTurret().getHeightOfLauncher());

        atVision.initAprilTag();
    }

    @Override
    public void init_loop(){

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
                RobotDataV2.setStartColor(ColorRange.BLUE);
                robotData.selectedColor();

                //openCVData.createColorLocator(RobotData.getStartColor());
                //openCVData.createVisionPortal();
            }

            //Right Side
            else if (gamepad1.b) {
                RobotDataV2.setStartColor(ColorRange.RED);
                robotData.selectedColor();

                //openCVData.createColorLocator(RobotData.getStartColor());
                //openCVData.createVisionPortal();
            }
        }

        else if (!robotData.isPendingColor() && robotData.isPendingPosition()){
            //Visual Display
            if (robotData.getOpenCVEnabled()){
                if (RobotDataV2.getStartColor().equals(ColorRange.BLUE)){
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
                RobotDataV2.setStartedLeft(true);
                rrData.setBeginPose(new Pose2d(0, 0, Math.toRadians(0)));
                rrData.createDrive();   //Created RoadRunner Robot Object (Mechanum Drive)
                robotData.selectedPosition();

                //Create Left Side Trajectory

                //Common Positions
                Pose2d spawn = rrData.getBeginPose();

                //Each Route

                //Possibly just move this to rrData Class
                TrajectoryActionBuilder test1 = rrData.getDrive().actionBuilder(spawn)
                        .lineToX(5);

                TrajectoryActionBuilder test2 = rrData.getDrive().actionBuilder(spawn)
                        .lineToY(5);

                //Insert Trajectory Paths Here
                rrData.createTrajectoryPath(new TrajectoryActionBuilder[]{test1,test2});
            }

            //Right Side
            else if (gamepad1.dpad_right){
                RobotDataV2.setStartedLeft(false);
                rrData.setBeginPose(new Pose2d(0, 0, Math.toRadians(0)));
                rrData.createDrive();   //Created RoadRunner Robot Object (Mechanum Drive)
                robotData.selectedPosition();

                //Create Right Side Trajectory

                //Common Positions
                Pose2d spawn = rrData.getBeginPose();

                //Each Route

                TrajectoryActionBuilder test1 = rrData.getDrive().actionBuilder(spawn)
                        .lineToX(5);

                TrajectoryActionBuilder test2 = rrData.getDrive().actionBuilder(spawn)
                        .lineToY(-5);

                //Insert Trajectory Paths Here
                rrData.createTrajectoryPath(new TrajectoryActionBuilder[]{test1,test2});
            }
        }

        else{
            telemetry.addLine("Completed Initialization.... \n --------------------------" );
            telemetry.addData("Chosen Color", RobotDataV2.getStartColor());
            telemetry.addData("Chosen Position", RobotDataV2.getStartingPosition());
            atVision.telemetryAprilTag();
            telemetry.update();
        }


    }

    @Override
    public void start(){
        //RobotData.setAutoRun(true);
        RobotDataV2.createRuntime();

        //Left Side Auto
        if (RobotDataV2.getStartedLeft()){
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

    @Override
    public void loop(){

    }

    @Override
    public void stop() {

        telemetry.addLine("Autonomous Completed!");
        telemetry.addData("Time Spent: ", RobotDataV2.getRuntime());

        if (robotData.getOpenCVEnabled()){
            //openCVData.closePortal();
        }

        atVision.closeVisionPortal();
    }
}