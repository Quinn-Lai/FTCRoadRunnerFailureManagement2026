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

        if (robotData.isPendingSide()){
            telemetry.addLine("Select a Starting Side!");
            telemetry.update();

            //Close
            if (gamepad1.dpad_up){
                robotData.setStartFar(false);
                robotData.selectedSide();
            }

            //Far Side
            else if (gamepad1.dpad_down){
                robotData.setStartFar(true);
                robotData.selectedSide();
            }

        }

        else if (robotData.isPendingPosition()){

            telemetry.addLine("Select a Starting Position!");
            telemetry.update();

            //Left Side
            if (gamepad1.dpad_left){
                robotData.setStartLeft(true);
                robotData.selectedPosition();
            }

            //Right Side
            else if (gamepad1.dpad_right){
                robotData.setStartLeft(false);
                robotData.selectedPosition();
            }
        }

        else{

            if (robotData.isAwaitingTrajectoryGeneration()){
                robotData.generatedTrajectory();

                //Shared Pathways
                Pose2d lineBot = new Pose2d(34, -28, Math.toRadians(270));
                Pose2d lineBotCollect = new Pose2d(34, -55, Math.toRadians(270));
                Pose2d lineMid = new Pose2d(11, -28, Math.toRadians(270));
                Pose2d lineMidCollect = new Pose2d(11, -55, Math.toRadians(270));
                Pose2d lineTop = new Pose2d(-12, -28, Math.toRadians(270));
                Pose2d lineTopCollect = new Pose2d(-12, -51, Math.toRadians(270));

                //Trajectory Left
                if (robotData.isStartedLeft()){
                    if (robotData.isStartFar()){
                        rrData.setBeginPose(new Pose2d(57, -10, Math.toRadians(180)));
                        rrData.createDrive();

                        //Common Positions
                        Pose2d spawn = rrData.getBeginPose();
                        Pose2d shootingPos = new Pose2d(57, -10, Math.toRadians(200));

                        //Each Route

                        //Possibly just move this to rrData Class
                        TrajectoryActionBuilder test1 = rrData.getDrive().actionBuilder(spawn)
                                .lineToX(5);

                        TrajectoryActionBuilder test2 = rrData.getDrive().actionBuilder(spawn)
                                .lineToY(5);

                        //Insert Trajectory Paths Here
                        rrData.createTrajectoryPath(new TrajectoryActionBuilder[]{test1,test2});
                    }
                    else{
                        rrData.setBeginPose(new Pose2d(0, 0, Math.toRadians(0)));
                        rrData.createDrive();   //Created RoadRunner Robot Object (Mechanum Drive)

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
                }

                //Trajectory Right
                else{
                    if (robotData.isStartFar()){
                        rrData.setBeginPose(new Pose2d(0, 0, Math.toRadians(0)));
                        rrData.createDrive();   //Created RoadRunner Robot Object (Mechanum Drive)

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
                    else{
                        rrData.setBeginPose(new Pose2d(0, 0, Math.toRadians(0)));
                        rrData.createDrive();   //Created RoadRunner Robot Object (Mechanum Drive)

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
                }
            }

            telemetry.addLine("Completed Initialization..... \n --------------------------" );
            telemetry.addData("Starting Position", robotData.getStartingPosition());
            telemetry.update();
        }

    }

    @Override
    public void start(){
        //RobotData.setAutoRun(true);
        //RobotDataV2.createRuntime();

        if (!limeLight.foundMotif()){
            LimeLightVision.failsafeMotif();
        }

        //Left Side Auto
        //TODO actually no matter what, all autos should follow the same pattern

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