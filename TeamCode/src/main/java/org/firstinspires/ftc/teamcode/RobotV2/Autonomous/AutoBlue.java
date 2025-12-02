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

        robotData = new RobotDataV2(hardwareMap, telemetry);       //Basic Robot Mechanics
        rrData = new RoadRunnerDataV2(robotData);  //Road Runner Implementation
        limeLight = new LimeLightVision(hardwareMap,telemetry,"blue");
        limeLight.updateAtHeight(robotData.getTurret().getHeightOfLauncher());
        limeLight.initLimeLight();

        LimeLightVision.isFoundMotif = false;

        robotData.getCarosel().forceTransferDown();
    }

    @Override
    public void init_loop(){

        //Color Selection & OpenCV

        limeLight.updateOrientationIMU();
        limeLight.updateMotifCode();

        if (robotData.isPendingSide()){
            telemetry.addLine("Select a Starting Side (Far or Close)!");
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

            telemetry.addLine("Select a Starting Position (Left or Right)!");
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

                //Trajectory Left
                if (robotData.isStartedLeft()){

                    //Shared Left

                    Pose2d lineBot = new Pose2d(34, -28, Math.toRadians(270));
                    Pose2d lineBotCollect = new Pose2d(34, -55, Math.toRadians(270));
                    Pose2d lineMid = new Pose2d(11, -28, Math.toRadians(270));
                    Pose2d lineMidCollect = new Pose2d(11, -55, Math.toRadians(270));
                    Pose2d lineTop = new Pose2d(-12, -28, Math.toRadians(270));
                    Pose2d lineTopCollect = new Pose2d(-12, -51, Math.toRadians(270));

                    if (robotData.isStartFar()){
                        rrData.setBeginPose(new Pose2d(61.065, -13, Math.toRadians(180)));
                        rrData.createDrive();

                        //Common Positions
                        Pose2d spawn = rrData.getBeginPose();
                        Pose2d shootingPos = new Pose2d(51, -13, Math.toRadians(205));

                        //Each Route

                        //Possibly just move this to rrData Class

                        TrajectoryActionBuilder shootFirst = rrData.getDrive().actionBuilder(spawn)
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(shootingPos,Math.toRadians(180));

                        TrajectoryActionBuilder bottomLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(200))
                                .splineToSplineHeading(lineBot, Math.toRadians(270));

                        TrajectoryActionBuilder bottomLineCollect = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(270))
                                .splineToSplineHeading(lineBotCollect, Math.toRadians(270));

                        TrajectoryActionBuilder shootSecond = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(shootingPos,Math.toRadians(20));

                        TrajectoryActionBuilder middleLinePrep = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(200))
                                .splineToLinearHeading(lineMid, Math.toRadians(180));

                        TrajectoryActionBuilder middleLineCollect = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(lineMidCollect, Math.toRadians(270));

                        TrajectoryActionBuilder shootThird = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(45))
                                .splineToLinearHeading(shootingPos,Math.toRadians(20));

                        TrajectoryActionBuilder topLinePrep = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(200))
                                .splineToLinearHeading(lineTop, Math.toRadians(180));

                        TrajectoryActionBuilder topLineCollect = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(lineTopCollect, Math.toRadians(270));

                        TrajectoryActionBuilder shootFourth = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(10))
                                .splineToLinearHeading(shootingPos,Math.toRadians(20));

                        //Insert Trajectory Paths Here
                        rrData.createTrajectoryPath(new TrajectoryActionBuilder[]{
                                shootFirst,
                                bottomLinePrep,
                                bottomLineCollect,
                                shootSecond,
                                middleLinePrep,
                                middleLineCollect,
                                shootThird,
                                topLinePrep,
                                topLineCollect,
                                shootFourth
                        });
                    }
                    else{

                        rrData.setBeginPose(new Pose2d(61.065, -13, Math.toRadians(180))); //TODO find
                        rrData.createDrive();

                        //Common Positions
                        Pose2d spawn = rrData.getBeginPose();
                        Pose2d shootingPos = new Pose2d(51, -13, Math.toRadians(205)); //TODO find

                        //Each Route

                        //Possibly just move this to rrData Class

                        TrajectoryActionBuilder shootFirst = rrData.getDrive().actionBuilder(spawn)
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(shootingPos,Math.toRadians(180));

                        TrajectoryActionBuilder bottomLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(200))
                                .splineToSplineHeading(lineBot, Math.toRadians(270));

                        TrajectoryActionBuilder bottomLineCollect = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(270))
                                .splineToSplineHeading(lineBotCollect, Math.toRadians(270));

                        TrajectoryActionBuilder shootSecond = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(shootingPos,Math.toRadians(20));

                        TrajectoryActionBuilder middleLinePrep = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(200))
                                .splineToLinearHeading(lineMid, Math.toRadians(180));

                        TrajectoryActionBuilder middleLineCollect = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(lineMidCollect, Math.toRadians(270));

                        TrajectoryActionBuilder shootThird = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(45))
                                .splineToLinearHeading(shootingPos,Math.toRadians(20));

                        TrajectoryActionBuilder topLinePrep = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(200))
                                .splineToLinearHeading(lineTop, Math.toRadians(180));

                        TrajectoryActionBuilder topLineCollect = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(lineTopCollect, Math.toRadians(270));

                        TrajectoryActionBuilder shootFourth = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(10))
                                .splineToLinearHeading(shootingPos,Math.toRadians(20));

                        //Insert Trajectory Paths Here
                        rrData.createTrajectoryPath(new TrajectoryActionBuilder[]{
                                shootFirst,
                                bottomLinePrep,
                                bottomLineCollect,
                                shootSecond,
                                middleLinePrep,
                                middleLineCollect,
                                shootThird,
                                topLinePrep,
                                topLineCollect,
                                shootFourth
                        });

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

                limeLight.setLimelightLocalizier(rrData.getDrive().getLocalizerPinpoint());
            }

            telemetry.addLine("Completed Initialization..... \n --------------------------" );
            telemetry.addData("Starting Side", robotData.getStartingSide());
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
                            rrData.getTrajectory(2)//,
                            //rrData.doReminder(0.2)
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
                            rrData.getTrajectory(2)//,
                            //rrData.doReminder(1)
                    )
            );
        }
    }

    @Override
    public void loop(){

    }

    @Override
    public void stop() {

        RoadRunnerDataV2.lastAutoPosition = rrData.getDrive().getLocalizerPinpoint().getPose(); //Get last pos

        telemetry.addLine("Autonomous Completed!");
        telemetry.addData("Time Spent: ", RobotDataV2.getRuntime());
        limeLight.killLimeLight();
    }
}