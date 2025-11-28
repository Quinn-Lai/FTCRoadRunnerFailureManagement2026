package org.firstinspires.ftc.teamcode.RobotV2.Autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotV2.ClassData.AprilTagVisionV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.LimeLightVision;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RoadRunnerDataV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotDataV2;
import org.firstinspires.ftc.vision.opencv.ColorRange;

@Autonomous
public class AutoBlue extends OpMode {

    //Data Classes
    private LimeLightVision limeLight;
    private RobotDataV2 robotData;       //Basic Robot Mechanics
    private RoadRunnerDataV2 rrData;  //Road Runner Implementation

    @Override
    public void init(){

        limeLight = new LimeLightVision(hardwareMap,telemetry,"blue");
        robotData = new RobotDataV2(hardwareMap, telemetry);       //Basic Robot Mechanics
        rrData = new RoadRunnerDataV2(robotData);  //Road Runner Implementation
        limeLight.updateAtHeight(robotData.getTurret().getHeightOfLauncher());
    }

    @Override
    public void init_loop(){

        //Color Selection & OpenCV

        limeLight.updateMotifCode();

        //TODO NEED TO ADD CLOSE AND FAR SIDE

        if (robotData.isPendingPosition()){

            telemetry.addLine("Select a Starting Position!");
            telemetry.update();

            //Left Side
            if (gamepad1.dpad_left){
                robotData.setStartLeft(true);

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
                robotData.setStartLeft(false);
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
            telemetry.addLine("Completed Initialization..... \n --------------------------" );
            telemetry.addData("Starting Position", robotData.getStartingPosition());
            telemetry.update();
        }


    }

    @Override
    public void start(){
        //RobotData.setAutoRun(true);
        RobotDataV2.createRuntime();

        //Left Side Auto
        if (robotData.isStartedLeft()){
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
        limeLight.killLimeLight();
    }
}