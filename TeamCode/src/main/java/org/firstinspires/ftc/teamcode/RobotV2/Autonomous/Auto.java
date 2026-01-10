package org.firstinspires.ftc.teamcode.RobotV2.Autonomous;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotV2.ClassData.LimeLightVision;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RoadRunnerDataV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotConstantsV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotDataV2;

import java.util.ArrayList;


@Autonomous
public class Auto extends OpMode {

    //Data Classes
    private LimeLightVision limeLight;
    private RoadRunnerDataV2 rrData;

    private final boolean patternEnabled = true;
    private final boolean isWaitTurret = false; //false

    @Override
    public void init(){

        RobotConstantsV2.FAILSAFE_SUBMODE_TIMER = RobotConstantsV2.FAILSAFE_SUBMODE_TIMER_AUTO;
        RobotConstantsV2.COOLDOWN_SHOT = RobotConstantsV2.COOLDOWN_SHOT_AUTO; //Transfer Shot
        RobotConstantsV2.COOLDOWN_PRE_SHOT = RobotConstantsV2.COOLDOWN_PRE_SHOT_AUTO;

        rrData = new RoadRunnerDataV2(new RobotDataV2(hardwareMap, telemetry));
        limeLight = new LimeLightVision(hardwareMap,telemetry,"blue");

        rrData.createDashboard();

        limeLight.updateAtHeight(rrData.getRobotData().getTurret().getHeightOfLauncher());
        limeLight.initLimeLight();

        LimeLightVision.isFoundMotif = false;
        RobotConstantsV2.CAROSEL_GLOBAL_INCREMENT = 0;
        RobotConstantsV2.CAROSEL_TOUCHPAD = 0;

        rrData.getRobotData().getCarosel().forceTransferDown();
        rrData.getRobotData().getCarosel().setInventoryAuto();
        rrData.getRobotData().getCarosel().indicatorsInInit();
        rrData.getRobotData().getCarosel().cycleOrigin();
        rrData.getRobotData().getCarosel().updateCaroselEncoder();
    }

    @Override
    public void init_loop(){

        //Color Selection & OpenCV

        limeLight.updateOrientationIMU();
        limeLight.updateMotifCode();

        if (gamepad1.crossWasPressed()){
            limeLight.switchAlliance();
        }

        if (gamepad1.circleWasPressed()){
            rrData.switchOpenGateActive();
        }

        if (rrData.getRobotData().isPendingSide()){
            telemetry.addLine("Select a Starting Side (Far or Close)!");
            telemetry.update();

            //Close
            if (gamepad1.dpad_up){
                rrData.getRobotData().setStartFar(false);
                rrData.getRobotData().selectedSide();
                rrData.setDisplacement(RobotConstantsV2.CLOSE_BALL_DISTANCE);
            }
            //Far Side
            else if (gamepad1.dpad_down){
                rrData.getRobotData().setStartFar(true);
                rrData.getRobotData().selectedSide();
                rrData.setDisplacement(RobotConstantsV2.FAR_BALL_DISTANCE);
            }

        }

        else if (rrData.getRobotData().isPendingPosition()){

            telemetry.addLine("Select a Starting Position (Left or Right)!");
            telemetry.update();

            //Left Side
            if (gamepad1.dpad_left){
                rrData.getRobotData().setStartLeft(true);
                rrData.getRobotData().selectedPosition();
            }

            //Right Side
            else if (gamepad1.dpad_right){
                rrData.getRobotData().setStartLeft(false);
                rrData.getRobotData().selectedPosition();
            }
        }

        else{

            if (rrData.getRobotData().isAwaitingTrajectoryGeneration()){

                rrData.getRobotData().generatedTrajectory();

                //Trajectory Left
                if (rrData.getRobotData().isStartedLeft()){

                    Pose2d lineBot = new Pose2d(33, -28, Math.toRadians(270));
                    Pose2d lineBotCollect = new Pose2d(33, -RobotConstantsV2.INTAKE_TRAVEL, Math.toRadians(270));
                    Pose2d lineMid = new Pose2d(11, -28, Math.toRadians(270));
                    Pose2d lineMidCollect = new Pose2d(11, -RobotConstantsV2.INTAKE_TRAVEL, Math.toRadians(270));
                    Pose2d lineTop = new Pose2d(-12, -28, Math.toRadians(270));
                    Pose2d lineTopCollect = new Pose2d(-12, -RobotConstantsV2.INTAKE_TRAVEL, Math.toRadians(270));

                    Pose2d collectFromGateClose = new Pose2d(0,0,0);
                    Pose2d collectFromGateFar = new Pose2d(0,0,0);
                    Pose2d gate = RobotConstantsV2.GATE_OPEN_POSITION_BLUE;

                    //Far Side
                    if (rrData.getRobotData().isStartFar()){

                        rrData.setIntendedHeading(RobotConstantsV2.PINPOINT_HEADING_FAR_BLUE);

                        rrData.setBeginPose(new Pose2d(61.065, -13, Math.toRadians(RobotConstantsV2.PINPOINT_HEADING_FAR_BLUE)));
                        rrData.createDrive();

                        //Common Positions
                        Pose2d spawn = rrData.getBeginPose();
                        Pose2d shootingPos = new Pose2d(51, -13, Math.toRadians(200));

                        //Each Route

                        TrajectoryActionBuilder shootFirst = rrData.getDrive().actionBuilder(spawn)
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(shootingPos,Math.toRadians(180));

                        TrajectoryActionBuilder bottomLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(200))
                                .splineToLinearHeading(lineBot, Math.toRadians(270),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                        TrajectoryActionBuilder bottomLineCollect = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(lineBotCollect, Math.toRadians(270), new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                        TrajectoryActionBuilder shootSecond = rrData.getDrive().actionBuilder(lineBotCollect)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(shootingPos,Math.toRadians(20), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                        TrajectoryActionBuilder middleLinePrep;
                        TrajectoryActionBuilder middleLineCollect;
                        TrajectoryActionBuilder shootThird;

                        if (rrData.isOpenGateActive()){
                            middleLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                    .setTangent(Math.toRadians(180))
                                    .splineToLinearHeading(lineMid, Math.toRadians(270),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                            middleLineCollect = rrData.getDrive().actionBuilder(lineMid)
                                    .setTangent(Math.toRadians(270))
                                    .splineToLinearHeading(gate, Math.toRadians(270), new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                            shootThird = rrData.getDrive().actionBuilder(gate)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(shootingPos,Math.toRadians(0),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));
                        }

                        else{
                            middleLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                    .setTangent(Math.toRadians(200))
                                    .splineToLinearHeading(lineMid, Math.toRadians(180),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                            middleLineCollect = rrData.getDrive().actionBuilder(lineMid)
                                    .setTangent(Math.toRadians(270))
                                    .splineToLinearHeading(lineMidCollect, Math.toRadians(270), new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                            shootThird = rrData.getDrive().actionBuilder(lineMidCollect)
                                    .setTangent(Math.toRadians(45))
                                    .splineToLinearHeading(shootingPos,Math.toRadians(20),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));
                        }

                        TrajectoryActionBuilder topLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(200))
                                .splineToLinearHeading(lineTop, Math.toRadians(180), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                        TrajectoryActionBuilder topLineCollect = rrData.getDrive().actionBuilder(lineTop)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(lineTopCollect, Math.toRadians(270), new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                        TrajectoryActionBuilder shootFourth = rrData.getDrive().actionBuilder(lineTopCollect)
                                .setTangent(Math.toRadians(10))
                                .splineToLinearHeading(shootingPos,Math.toRadians(20), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

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
                                shootFourth,
                        });
                    }

                    //Close Side
                    else{
                        rrData.setIntendedHeading(RobotConstantsV2.PINPOINT_HEADING_CLOSE_BLUE);

                        //new Pose2d(-52, -48.5, Math.toRadians(RobotConstantsV2.PINPOINT_HEADING_CLOSE_BLUE))
                        rrData.setBeginPose(new Pose2d(-49.5, -50, Math.toRadians(RobotConstantsV2.PINPOINT_HEADING_CLOSE_BLUE)));
                        rrData.createDrive();

                        //Common Positions
                        Pose2d spawn = rrData.getBeginPose();
                        Pose2d shootingPos = new Pose2d(-15, -15, Math.toRadians(225));

                        //Each Route

                        //30
                        TrajectoryActionBuilder shootFirst = rrData.getDrive().actionBuilder(spawn)
                                .setTangent(Math.toRadians(45))
                                .splineToLinearHeading(shootingPos,Math.toRadians(45),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                        TrajectoryActionBuilder topLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(315))
                                .splineToLinearHeading(lineTop, Math.toRadians(270));


                        TrajectoryActionBuilder topLineCollect = rrData.getDrive().actionBuilder(lineTop)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(lineTopCollect,Math.toRadians(270),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));


                        TrajectoryActionBuilder shootSecond = rrData.getDrive().actionBuilder(lineTopCollect)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(shootingPos,Math.toRadians(90));

                        TrajectoryActionBuilder middleLinePrep;
                        TrajectoryActionBuilder middleLineCollect;
                        TrajectoryActionBuilder shootThird;

                        if (rrData.isOpenGateActive()){
                            middleLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                    .setTangent(Math.toRadians(0))
                                    .splineToLinearHeading(lineMid, Math.toRadians(270));

                            middleLineCollect = rrData.getDrive().actionBuilder(lineMid)
                                    .setTangent(Math.toRadians(270))
                                    .splineToLinearHeading(gate,Math.toRadians(270),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                            shootThird = rrData.getDrive().actionBuilder(gate)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(shootingPos,Math.toRadians(180));
                        }

                        else{
                            middleLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                    .setTangent(Math.toRadians(0))
                                    .splineToLinearHeading(lineMid, Math.toRadians(270));

                            middleLineCollect = rrData.getDrive().actionBuilder(lineMid)
                                    .setTangent(Math.toRadians(270))
                                    .splineToLinearHeading(lineMidCollect,Math.toRadians(270),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                            shootThird = rrData.getDrive().actionBuilder(lineMidCollect)
                                    .setTangent(Math.toRadians(135))
                                    .splineToLinearHeading(shootingPos,Math.toRadians(90));
                        }

                        TrajectoryActionBuilder bottomLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(lineBot, Math.toRadians(270));

                        TrajectoryActionBuilder bottomLineCollect = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(lineBotCollect,Math.toRadians(270),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                        TrajectoryActionBuilder shootFourth = rrData.getDrive().actionBuilder(lineBotCollect)
                                .setTangent(Math.toRadians(135))
                                .splineToLinearHeading(shootingPos,Math.toRadians(135));

                        //Insert Trajectory Paths Here
                        rrData.createTrajectoryPath(new TrajectoryActionBuilder[]{
                                shootFirst,
                                topLinePrep,
                                topLineCollect,
                                shootSecond,
                                middleLinePrep,
                                middleLineCollect,
                                shootThird,
                                bottomLinePrep,
                                bottomLineCollect,
                                shootFourth
                        });
                    }
                }

                //Trajectory Right
                else{
                    Pose2d lineBot = new Pose2d(33, 28, Math.toRadians(90));
                    Pose2d lineBotCollect = new Pose2d(33, RobotConstantsV2.INTAKE_TRAVEL, Math.toRadians(90));
                    Pose2d lineMid = new Pose2d(11, 28, Math.toRadians(90));
                    Pose2d lineMidCollect = new Pose2d(11, RobotConstantsV2.INTAKE_TRAVEL, Math.toRadians(90));
                    Pose2d lineTop = new Pose2d(-12, 28, Math.toRadians(90));
                    Pose2d lineTopCollect = new Pose2d(-12, RobotConstantsV2.INTAKE_TRAVEL, Math.toRadians(90));

                    Pose2d collectFromGateClose = new Pose2d(0,0,0);
                    Pose2d collectFromGateFar = new Pose2d(0,0,0);
                    Pose2d gate = RobotConstantsV2.GATE_OPEN_POSITION_RED;

                    if (rrData.getRobotData().isStartFar()){
                        rrData.setIntendedHeading(RobotConstantsV2.PINPOINT_HEADING_FAR_RED);

                        rrData.setBeginPose( new Pose2d(61.065, 13, Math.toRadians(RobotConstantsV2.PINPOINT_HEADING_FAR_RED)));
                        rrData.createDrive();

                        //Common Positions
                        Pose2d spawn = rrData.getBeginPose();
                        Pose2d shootingPos = new Pose2d(51, 13, Math.toRadians(160));

                        //Each Route

                        TrajectoryActionBuilder shootFirst = rrData.getDrive().actionBuilder(spawn)
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(shootingPos,Math.toRadians(180));

                        TrajectoryActionBuilder bottomLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(150))
                                .splineToSplineHeading(lineBot, Math.toRadians(90),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                        TrajectoryActionBuilder bottomLineCollect = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(90))
                                .splineToSplineHeading(lineBotCollect, Math.toRadians(90), new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                        TrajectoryActionBuilder shootSecond = rrData.getDrive().actionBuilder(lineBotCollect)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(shootingPos,Math.toRadians(340),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                        TrajectoryActionBuilder middleLinePrep;
                        TrajectoryActionBuilder middleLineCollect;
                        TrajectoryActionBuilder shootThird;

                        if (rrData.isOpenGateActive()){
                            middleLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                    .setTangent(Math.toRadians(180))
                                    .splineToLinearHeading(lineMid, Math.toRadians(90),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                            middleLineCollect = rrData.getDrive().actionBuilder(lineMid)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(gate, Math.toRadians(90), new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                            shootThird = rrData.getDrive().actionBuilder(gate)
                                    .setTangent(Math.toRadians(270))
                                    .splineToLinearHeading(shootingPos,Math.toRadians(0),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));
                        }
                        else{
                            middleLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                    .setTangent(Math.toRadians(150))
                                    .splineToLinearHeading(lineMid, Math.toRadians(180),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                            middleLineCollect = rrData.getDrive().actionBuilder(lineMid)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(lineMidCollect, Math.toRadians(90), new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                            shootThird = rrData.getDrive().actionBuilder(lineMidCollect)
                                    .setTangent(Math.toRadians(315))
                                    .splineToLinearHeading(shootingPos,Math.toRadians(315),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));
                        }

                        TrajectoryActionBuilder topLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(150))
                                .splineToLinearHeading(lineTop, Math.toRadians(180),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                        TrajectoryActionBuilder topLineCollect = rrData.getDrive().actionBuilder(lineTop)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(lineTopCollect, Math.toRadians(90),  new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                        TrajectoryActionBuilder shootFourth = rrData.getDrive().actionBuilder(lineTopCollect)
                                .setTangent(Math.toRadians(350))
                                .splineToLinearHeading(shootingPos,Math.toRadians(340),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

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

                        //Close Side

                        rrData.setIntendedHeading(RobotConstantsV2.PINPOINT_HEADING_CLOSE_RED);

                        //new Pose2d(-52, 48.5, Math.toRadians(RobotConstantsV2.PINPOINT_HEADING_CLOSE_RED))
                        rrData.setBeginPose(new Pose2d(-49.5, 50, Math.toRadians(RobotConstantsV2.PINPOINT_HEADING_CLOSE_RED)));
                        rrData.createDrive();

                        //Common Positions
                        Pose2d spawn = rrData.getBeginPose();
                        Pose2d shootingPos = new Pose2d(-15, 15, Math.toRadians(135));

                        //Each Route

                        TrajectoryActionBuilder shootFirst = rrData.getDrive().actionBuilder(spawn)
                                .setTangent(Math.toRadians(315)) //330
                                .splineToLinearHeading(shootingPos,Math.toRadians(315), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                        TrajectoryActionBuilder topLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(45))
                                .splineToLinearHeading(lineTop, Math.toRadians(90));


                        TrajectoryActionBuilder topLineCollect = rrData.getDrive().actionBuilder(lineTop)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(lineTopCollect,Math.toRadians(90),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                        TrajectoryActionBuilder shootSecond = rrData.getDrive().actionBuilder(lineTopCollect)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(shootingPos,Math.toRadians(270));

                        TrajectoryActionBuilder middleLinePrep;
                        TrajectoryActionBuilder middleLineCollect;
                        TrajectoryActionBuilder shootThird;

                        if (rrData.isOpenGateActive()){
                            middleLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                    .setTangent(Math.toRadians(0))
                                    .splineToLinearHeading(lineMid, Math.toRadians(90));

                            middleLineCollect = rrData.getDrive().actionBuilder(lineMid)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(lineMidCollect,Math.toRadians(90),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                            shootThird = rrData.getDrive().actionBuilder(lineMidCollect)
                                    .setTangent(Math.toRadians(270))
                                    .splineToLinearHeading(shootingPos,Math.toRadians(180));
                        }

                        else{
                            middleLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                    .setTangent(Math.toRadians(0))
                                    .splineToLinearHeading(lineMid, Math.toRadians(90));

                            middleLineCollect = rrData.getDrive().actionBuilder(lineMid)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(lineMidCollect,Math.toRadians(90),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                            shootThird = rrData.getDrive().actionBuilder(lineMidCollect)
                                    .setTangent(Math.toRadians(225))
                                    .splineToLinearHeading(shootingPos,Math.toRadians(270));
                        }

                        TrajectoryActionBuilder bottomLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(lineBot, Math.toRadians(90));


                        TrajectoryActionBuilder bottomLineCollect = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(lineBotCollect,Math.toRadians(90),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                        TrajectoryActionBuilder shootFourth = rrData.getDrive().actionBuilder(lineBotCollect)
                                .setTangent(Math.toRadians(225))
                                .splineToLinearHeading(shootingPos,Math.toRadians(225));


                        //Insert Trajectory Paths Here
                        rrData.createTrajectoryPath(new TrajectoryActionBuilder[]{
                                shootFirst,
                                topLinePrep,
                                topLineCollect,
                                shootSecond,
                                middleLinePrep,
                                middleLineCollect,
                                shootThird,
                                bottomLinePrep,
                                bottomLineCollect,
                                shootFourth
                        });


                    }
                }

                limeLight.setLimelightLocalizier(rrData.getDrive().getLocalizerPinpoint());
                rrData.getRobotData().getCarosel().indicatorsInInit();
            }

            telemetry.addLine("Completed Initialization..... \n --------------------------" );

            telemetry.addData("Found Motif", limeLight.foundMotif());
            if (LimeLightVision.isFoundMotif){
                rrData.getRobotData().getCarosel().updatePattern(LimeLightVision.motifCode);
                telemetry.addData("Motif: ", LimeLightVision.motifCode[0] + ", " + LimeLightVision.motifCode[1] + ", " + LimeLightVision.motifCode[2]);
            }

//            telemetry.addData("Pinpoint Yaw: ", rrData.getDrive().getLocalizerPinpoint().getHeadingLocalizerDegrees());
//            telemetry.addData("Pinpoint Heading (diff): ", rrData.getDrive().getLocalizerPinpoint().getPose().heading);
//            telemetry.addData("Pinpoint x: ", rrData.getDrive().getLocalizerPinpoint().getPose().position.x);
            telemetry.addLine(String.format("Inventory: (%s, %s, %s)", rrData.getRobotData().getCarosel().getInventory()[0], rrData.getRobotData().getCarosel().getInventory()[1], rrData.getRobotData().getCarosel().getInventory()[2]));
            telemetry.addData("Starting Side", rrData.getRobotData().getStartingSide());
            telemetry.addData("Starting Position", rrData.getRobotData().getStartingPosition());
            telemetry.addData("Starting Color: ", limeLight.getAlliance());
            telemetry.addData("Pin Point Heading: ", rrData.getDrive().getLocalizerPinpoint().getHeadingLocalizerDegrees());
            telemetry.addData("Pin Point Working: ", rrData.isPinpointHeadingCorrect(rrData.getDrive().getLocalizerPinpoint().getHeadingLocalizerDegrees()));
            telemetry.addData("Attempt Gate Open: ", rrData.isOpenGateActive());

            telemetry.update();
        }

    }

    @Override
    public void start(){
        //RobotData.setAutoRun(true);
        //RobotDataV2.createRuntime();

        if (!LimeLightVision.isFoundMotif){
            LimeLightVision.failsafeMotif();
        }

        //Far side Auto

        if (rrData.getRobotData().isStartFar()){
            telemetry.addLine("Beginning Auto: Far Side");
            telemetry.update();
            rrData.getRobotData().getTurret().toggleTurretFar(true);

        }

        //Close Side Auto
        else{
            telemetry.addLine("Beginning Auto: Close Side");
            telemetry.update();
            rrData.getRobotData().getTurret().toggleTurretFar(false);
        }

        rrData.setLoopStatus(true);

        SequentialAction collectFirstLine = new SequentialAction(
                rrData.getTrajectory(2),
                rrData.intakeOn(),
                rrData.startFailsafeTimer(),
                new ParallelAction(
                        rrData.getTrajectory(3),
                        rrData.checkAutoIntake()
                ),
                rrData.intakeReverse(),
                rrData.cycleFirstPattern()
        );

        SequentialAction shootFirstLine = new SequentialAction(
                rrData.getTrajectory(4),
                rrData.intakeOff(),
                rrData.waitForTurret(isWaitTurret),
                rrData.requestArtifactShots(patternEnabled),
                rrData.shootArtifacts(rrData.getDisplacement(), patternEnabled),
                rrData.forceTransferDown()
        );

        SequentialAction collectSecondLine = new SequentialAction(
                rrData.getTrajectory(5),
                rrData.intakeOn(),
                rrData.startFailsafeTimer(),
                new ParallelAction(
                        rrData.getTrajectory(6),
                        rrData.checkAutoIntake()
                ),
                rrData.intakeReverse(),
                rrData.cycleFirstPattern()
        );

        SequentialAction shootSecondLine = new SequentialAction(
                rrData.getTrajectory(7),
                rrData.intakeOff(),
                rrData.waitForTurret(isWaitTurret),
                rrData.requestArtifactShots(patternEnabled),
                rrData.shootArtifacts(rrData.getDisplacement(), patternEnabled),
                rrData.forceTransferDown()
        );

        SequentialAction collectThirdLine = new SequentialAction(
                rrData.getTrajectory(8),
                rrData.intakeOn(),
                rrData.startFailsafeTimer(),
                new ParallelAction(
                        rrData.getTrajectory(9),
                        rrData.checkAutoIntake()
                ),
                rrData.intakeReverse(),
                rrData.cycleFirstPattern()
        );

        SequentialAction shootThirdLine = new SequentialAction(
                rrData.getTrajectory(10),
                rrData.intakeOff(),
                rrData.waitForTurret(isWaitTurret),
                rrData.requestArtifactShots(patternEnabled),
                rrData.shootArtifacts(rrData.getDisplacement(), patternEnabled),
                rrData.forceTransferDown(),
                rrData.cycleFirstPattern()
        );

        SequentialAction collectCloseGate = new SequentialAction(

        );

        SequentialAction collectFarGate = new SequentialAction(

        );

        SequentialAction[] buildBear;

        buildBear = new SequentialAction[]{
                collectFirstLine,   //0
                shootFirstLine,     //1
                collectSecondLine,  //2
                shootSecondLine,    //3
                collectThirdLine,   //4
                shootThirdLine,     //5
        };

        //For Gate
        if (rrData.isOpenGateActive()){
            buildBear = new SequentialAction[]{
                    collectSecondLine,  //2
                    shootSecondLine,    //3
                    collectFirstLine,   //0
                    shootFirstLine,     //1
                    collectThirdLine,   //4
                    shootThirdLine,     //5
            };
        }

        Actions.runBlocking(
                new ParallelAction(

                        //Repeated Stuff

                        //rrData.updateCaroselEncoder(),
                        rrData.indicatorsUpdate(limeLight),
                        //rrData.updateTelemetry(telemetry),
                        rrData.turretPID(),
                        rrData.locateAprilTag(limeLight),
                        rrData.telemetryAuto(),


                        new SequentialAction(

                                //Speed Up Init

                                new ParallelAction(
                                        rrData.getTrajectory(1),
                                        rrData.waitForTurret(isWaitTurret)
                                ),

                                //First Ball

                                rrData.requestArtifactShots(patternEnabled),
                                rrData.shootArtifacts(rrData.getDisplacement(), patternEnabled),
                                rrData.forceTransferDown(),

                                buildBear[0],
                                buildBear[1],
                                buildBear[2],
                                buildBear[3],
                                buildBear[4],
                                buildBear[5],

                                rrData.setLooping(false)
                        )
                )
        );

    //TODO op mode stop after if abort

    }

    @Override
    public void loop(){

    }

    @Override
    public void stop() {

        rrData.setLooping(false);

        //RoadRunnerDataV2.lastAutoPosition = rrData.getDrive().getLocalizerPinpoint().getPose(); //Get last pos

        telemetry.addLine("Autonomous Completed!");
        telemetry.addData("Time Spent: ", RobotDataV2.getRuntime());
        telemetry.update();

        limeLight.killLimeLight();
    }
}