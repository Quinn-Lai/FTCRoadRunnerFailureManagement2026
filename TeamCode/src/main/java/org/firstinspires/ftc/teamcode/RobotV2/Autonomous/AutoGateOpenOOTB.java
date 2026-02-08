package org.firstinspires.ftc.teamcode.RobotV2.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
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


@Config
@Autonomous
public class AutoGateOpenOOTB extends OpMode {

    //Data Classes
    private LimeLightVision limeLight;
    private RoadRunnerDataV2 rrData;

    private static boolean patternEnabled = true;
    private boolean isWaitTurret = false; //false
    private boolean forceFeedActive = true;

    @Override
    public void init(){

        RoadRunnerDataV2.isAutoPosStored = false;

        RobotConstantsV2.CAROSEL_TOLERANCE = RobotConstantsV2.CAROSEL_TOLERANCE_AUTO;
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

        rrData.getRobotData().setStartFar(false);
        rrData.getRobotData().selectedSide();
        rrData.setDisplacement(RobotConstantsV2.CLOSE_BALL_DISTANCE);
    }

    @Override
    public void init_loop(){

        //Color Selection & OpenCV

        limeLight.updateOrientationIMU(rrData.getYaw());
        limeLight.updateMotifCode();

        if (rrData.getRobotData().isPendingPosition()){

            telemetry.addLine("Select a Starting Position (Left or Right)!");
            telemetry.update();

            //Left Side
            if (gamepad1.dpad_left){
                limeLight.forceAllianceBlue();
                rrData.getRobotData().setStartLeft(true);
                rrData.getRobotData().selectedPosition();
            }

            //Right Side
            else if (gamepad1.dpad_right){
                limeLight.forceAllianceRed();
                rrData.getRobotData().setStartLeft(false);
                rrData.getRobotData().selectedPosition();
            }
        }

        else{

            if (rrData.getRobotData().isAwaitingTrajectoryGeneration()){

                rrData.getRobotData().generatedTrajectory();

                //Trajectory Left
                if (rrData.getRobotData().isStartedLeft()){

                    Pose2d lineBot = RobotConstantsV2.LINE_BOT_PREP_BLUE;
                    Pose2d lineBotCollect = RobotConstantsV2.LINE_BOT_COLLECT_BLUE;
                    Pose2d lineMid = RobotConstantsV2.LINE_MID_PREP_BLUE;
                    Pose2d lineMidCollect = new Pose2d(RobotConstantsV2.LINE_MID_PREP_TRAVEL, -42, Math.toRadians(270));
                    Pose2d lineTop = RobotConstantsV2.LINE_TOP_PREP_BLUE;
                    Pose2d lineTopCollect = new Pose2d(-RobotConstantsV2.LINE_TOP_PREP_TRAVEL, -43, Math.toRadians(270));

                    Pose2d collectFromGateClose = RobotConstantsV2.COLLECT_GATE_CLOSE_BLUE;
                    Pose2d collectFromGateFar = RobotConstantsV2.COLLECT_GATE_FAR_BLUE;
                    Pose2d gate = RobotConstantsV2.GATE_OPEN_POSITION_BLUE;

                    //Close Side

                    rrData.setIntendedHeading(RobotConstantsV2.PINPOINT_HEADING_CLOSE_BLUE);

                    rrData.setBeginPose(RobotConstantsV2.BLUE_SPAWN_CLOSE);
                    //rrData.setBeginPose(new Pose2d(-49.5, -50, Math.toRadians(RobotConstantsV2.PINPOINT_HEADING_CLOSE_BLUE)));
                    rrData.createDrive();

                    //Common Positions
                    Pose2d spawn = rrData.getBeginPose();
                    Pose2d shootingPos = RobotConstantsV2.BLUE_SHOOT_CLOSE;

                    //Each Route

                    TrajectoryActionBuilder shootFirst = rrData.getDrive().actionBuilder(spawn)
                            .setTangent(Math.toRadians(30))
                            .splineToLinearHeading(shootingPos,Math.toRadians(30),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

                    TrajectoryActionBuilder topLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                            .setTangent(Math.toRadians(315))
                            .splineToLinearHeading(lineTop, Math.toRadians(270), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

                    TrajectoryActionBuilder topLineCollect = rrData.getDrive().actionBuilder(lineTop)
                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(lineTopCollect,Math.toRadians(270),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SUPER_SLOW_SPEED))

                            .setTangent(Math.toRadians(0))
                            .splineToLinearHeading(RobotConstantsV2.SLAM_GATE_BLUE, Math.toRadians(90), new TranslationalVelConstraint(RobotConstantsV2.AUTO_GATE_SPEED));

                    TrajectoryActionBuilder shootSecond = rrData.getDrive().actionBuilder(RobotConstantsV2.SLAM_GATE_BLUE)
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(shootingPos,Math.toRadians(90), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED),  new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

                    TrajectoryActionBuilder middleLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                            .setTangent(Math.toRadians(0))
                            .splineToLinearHeading(lineMid, Math.toRadians(270), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED),new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

                    TrajectoryActionBuilder middleLineCollect = rrData.getDrive().actionBuilder(lineMid)
                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(lineMidCollect,Math.toRadians(270),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SUPER_SLOW_SPEED))

                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(gate,Math.toRadians(270),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SUPER_SLOW_SPEED));
//                        TrajectoryActionBuilder shootThird = rrData.getDrive().actionBuilder(gate)
//                                .setTangent(Math.toRadians(270))
//                                .splineToLinearHeading(lineMid, Math.toRadians(90), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED))
//
//                                .setTangent(Math.toRadians(90))
//                                .splineToLinearHeading(shootingPos,Math.toRadians(180), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));
//
                    TrajectoryActionBuilder shootThird = rrData.getDrive().actionBuilder(gate)
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(shootingPos,Math.toRadians(180), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

                    TrajectoryActionBuilder bottomLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                            .setTangent(Math.toRadians(0))
                            .splineToLinearHeading(lineBot, Math.toRadians(270), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

                    TrajectoryActionBuilder bottomLineCollect = rrData.getDrive().actionBuilder(lineBot)
                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(lineBotCollect,Math.toRadians(270),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

                    TrajectoryActionBuilder shootFourth = rrData.getDrive().actionBuilder(lineBotCollect)
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(RobotConstantsV2.OFF_LINE_BLUE_SHOOT_CLOSE,Math.toRadians(135), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

                    TrajectoryActionBuilder parkGate = rrData.getDrive().actionBuilder(RobotConstantsV2.OFF_LINE_BLUE_SHOOT_CLOSE)
                            .setTangent(Math.toRadians(0))
                            .splineToLinearHeading(RobotConstantsV2.PARK_GATE_BLUE,Math.toRadians(270), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));


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
                            shootFourth,
                            parkGate
                    });

                }

                //Trajectory Right
                else{
                    Pose2d lineBot = RobotConstantsV2.LINE_BOT_PREP_RED;
                    Pose2d lineBotCollect = RobotConstantsV2.LINE_BOT_COLLECT_RED;
                    Pose2d lineMid = RobotConstantsV2.LINE_MID_PREP_RED;
                    Pose2d lineMidCollect = new Pose2d(RobotConstantsV2.LINE_MID_PREP_TRAVEL, 42, Math.toRadians(90));
                    Pose2d lineTop = RobotConstantsV2.LINE_TOP_PREP_RED;
                    Pose2d lineTopCollect = new Pose2d(-RobotConstantsV2.LINE_TOP_PREP_TRAVEL, 43, Math.toRadians(90));;

                    Pose2d collectFromGateClose = RobotConstantsV2.COLLECT_GATE_CLOSE_RED;
                    Pose2d collectFromGateFar = RobotConstantsV2.COLLECT_GATE_FAR_RED;
                    Pose2d gate = RobotConstantsV2.GATE_OPEN_POSITION_RED;

                    //Close Side

                    rrData.setIntendedHeading(RobotConstantsV2.PINPOINT_HEADING_CLOSE_RED);

                    rrData.setBeginPose(RobotConstantsV2.RED_SPAWN_CLOSE);
                    //rrData.setBeginPose(new Pose2d(-49.5, 50, Math.toRadians(RobotConstantsV2.PINPOINT_HEADING_CLOSE_RED)));
                    rrData.createDrive();

                    //Common Positions
                    Pose2d spawn = rrData.getBeginPose();
                    Pose2d shootingPos = RobotConstantsV2.RED_SHOOT_CLOSE;

                    //Each Route

                    TrajectoryActionBuilder shootFirst = rrData.getDrive().actionBuilder(spawn)
                            .setTangent(Math.toRadians(330))
                            .splineToLinearHeading(shootingPos,Math.toRadians(330), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

                    TrajectoryActionBuilder topLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                            .setTangent(Math.toRadians(45))
                            .splineToLinearHeading(lineTop, Math.toRadians(90), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

                    TrajectoryActionBuilder topLineCollect = rrData.getDrive().actionBuilder(lineTop)
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(lineTopCollect,Math.toRadians(90),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SUPER_SLOW_SPEED))

                            .setTangent(Math.toRadians(0))
                            .splineToLinearHeading(RobotConstantsV2.SLAM_GATE_RED, Math.toRadians(270), new TranslationalVelConstraint(RobotConstantsV2.AUTO_GATE_SPEED));

                    TrajectoryActionBuilder shootSecond = rrData.getDrive().actionBuilder(RobotConstantsV2.SLAM_GATE_RED)
                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(shootingPos,Math.toRadians(270), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

                    TrajectoryActionBuilder middleLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                            .setTangent(Math.toRadians(0))
                            .splineToLinearHeading(lineMid, Math.toRadians(90), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

                    TrajectoryActionBuilder middleLineCollect = rrData.getDrive().actionBuilder(lineMid)
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(lineMidCollect,Math.toRadians(90),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SUPER_SLOW_SPEED))

                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(gate,Math.toRadians(90),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SUPER_SLOW_SPEED));
//                        TrajectoryActionBuilder shootThird = rrData.getDrive().actionBuilder(gate)
//                                .setTangent(Math.toRadians(270))
//                                .splineToLinearHeading(lineMid, Math.toRadians(270), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED))
//
//                                .setTangent(Math.toRadians(270))
//                                .splineToLinearHeading(shootingPos,Math.toRadians(180), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED)
                    TrajectoryActionBuilder shootThird = rrData.getDrive().actionBuilder(gate)
                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(shootingPos,Math.toRadians(180), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

                    TrajectoryActionBuilder bottomLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                            .setTangent(Math.toRadians(0))
                            .splineToLinearHeading(lineBot, Math.toRadians(90), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

                    TrajectoryActionBuilder bottomLineCollect = rrData.getDrive().actionBuilder(lineBot)
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(lineBotCollect,Math.toRadians(90),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

                    TrajectoryActionBuilder shootFourth = rrData.getDrive().actionBuilder(lineBotCollect)
                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(RobotConstantsV2.OFF_LINE_RED_SHOOT_CLOSE,Math.toRadians(225), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

                    TrajectoryActionBuilder parkGate = rrData.getDrive().actionBuilder(RobotConstantsV2.OFF_LINE_RED_SHOOT_CLOSE)
                            .setTangent(Math.toRadians(0))
                            .splineToLinearHeading(RobotConstantsV2.PARK_GATE_RED,Math.toRadians(90), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

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
                            shootFourth,
                            parkGate
                    });
                }

                //limeLight.setLimelightLocalizier(rrData.getDrive().getLocalizerPinpoint());
                rrData.getRobotData().getCarosel().indicatorsInInit();
            }

            telemetry.addLine("Completed Initialization..... \n --------------------------" );

            telemetry.addData("Found Motif", limeLight.foundMotif());
            if (LimeLightVision.isFoundMotif){
                rrData.getRobotData().getCarosel().updatePattern(LimeLightVision.motifCode);
                telemetry.addData("Motif: ", LimeLightVision.motifCode[0] + ", " + LimeLightVision.motifCode[1] + ", " + LimeLightVision.motifCode[2]);
            }

            telemetry.addLine(String.format("Inventory: (%s, %s, %s)", rrData.getRobotData().getCarosel().getInventory()[0], rrData.getRobotData().getCarosel().getInventory()[1], rrData.getRobotData().getCarosel().getInventory()[2]));
            telemetry.addData("Starting Side", rrData.getRobotData().getStartingSide());
            telemetry.addData("Starting Position", rrData.getRobotData().getStartingPosition());
            telemetry.addData("Starting Color: ", limeLight.getAlliance());
            //telemetry.addData("Pin Point Heading: ", rrData.getDrive().getLocalizerPinpoint().getHeadingLocalizerDegrees());
            telemetry.addData("Pin Point Working: ", rrData.isPinpointHeadingCorrect(rrData.getDrive().getLocalizerPinpoint().getHeadingLocalizerDegrees()));

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
        rrData.getRobotData().getTurret().toggleTurretFar(false);

        SequentialAction collectFirstLine = new SequentialAction(
                rrData.forceFeedCycle(forceFeedActive),
                rrData.getTrajectory(2),
                rrData.intakeOn(),
                rrData.startFailsafeTimer(),
                new ParallelAction(
                        rrData.getTrajectory(3),
                        rrData.checkAutoIntake()
                ),
                rrData.intakeOff(),
                rrData.forceFeedInventory(forceFeedActive, "Purple","Purple", "Green"),
                rrData.cycleQuickSlot(patternEnabled)
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
                rrData.forceFeedCycle(forceFeedActive),
                rrData.getTrajectory(5),
                rrData.intakeOn(),
                rrData.startFailsafeTimer(),
                new ParallelAction(
                        rrData.getTrajectory(6),
                        rrData.checkAutoIntake()
                ),
                rrData.intakeReverse(),
                rrData.forceFeedInventory(forceFeedActive, "Purple","Green","Purple"),
                rrData.cycleQuickSlot(patternEnabled)
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
                rrData.forceFeedCycle(forceFeedActive),
                rrData.getTrajectory(8),
                rrData.intakeOn(),
                rrData.startFailsafeTimer(),
                new ParallelAction(
                        rrData.getTrajectory(9),
                        rrData.checkAutoIntake()
                ),
                rrData.intakeReverse(),
                rrData.forceFeedInventory(forceFeedActive,"Green","Purple","Purple"),
                rrData.cycleQuickSlot(patternEnabled)
        );

        SequentialAction shootThirdLine = new SequentialAction(
                rrData.getTrajectory(10),
                rrData.intakeOff(),
                rrData.waitForTurret(isWaitTurret),
                rrData.requestArtifactShots(patternEnabled),
                rrData.shootArtifacts(rrData.getDisplacement(), patternEnabled),
                rrData.forceTransferDown(),
                rrData.cycleQuickSlot(patternEnabled)
        );

        SequentialAction collectCloseGate = new SequentialAction(

        );

        SequentialAction collectFarGate = new SequentialAction(

        );

        SequentialAction[] buildBear = new SequentialAction[]{
                    collectFirstLine,   //0
                    shootFirstLine,     //1
                    collectSecondLine,  //2
                    shootSecondLine,    //3
                    collectThirdLine,   //4
                    shootThirdLine,     //5
            };

        rrData.setLoopStatus(true);

        Actions.runBlocking(
                new ParallelAction(

                        //Repeated Stuff

                        //rrData.updateCaroselEncoder(),
                        rrData.indicatorsUpdate(limeLight),
                        //rrData.updateInveentory(),
                        rrData.turretPID(),
                        rrData.locateAprilTag(limeLight),
                        rrData.telemetryAuto(limeLight),

                        new SequentialAction(

                                //Speed Up Init

                                new ParallelAction(
                                        rrData.getTrajectory(1),
                                        rrData.waitForTurret(isWaitTurret)
                                ),

                                //First Ball

                                rrData.requestArtifactShots(false),
                                rrData.shootArtifacts(rrData.getDisplacement(), false),
                                rrData.forceTransferDown(),

                                buildBear[0],
                                buildBear[1],
                                buildBear[2],
                                buildBear[3],
                                buildBear[4],
                                buildBear[5],

                                rrData.setLooping(false),

                                rrData.killTurret(),
                                rrData.getTrajectory(11),
                                rrData.killTurret()

                        )
                )
        );
    }

    @Override
    public void loop(){

    }

    @Override
    public void stop() {

        RoadRunnerDataV2.isAutoPosStored = true; //TODO rememeber
        RoadRunnerDataV2.lastAutoPosition = rrData.getDrive().localizer.getPose(); //Get last pos

        rrData.setLooping(false);

        //RoadRunnerDataV2.lastAutoPosition = rrData.getDrive().getLocalizerPinpoint().getPose(); //Get last pos

        telemetry.addLine("Autonomous Completed!");
        telemetry.addData("Time Spent: ", RobotDataV2.getRuntime());
        telemetry.update();

        limeLight.killLimeLight();
    }
}