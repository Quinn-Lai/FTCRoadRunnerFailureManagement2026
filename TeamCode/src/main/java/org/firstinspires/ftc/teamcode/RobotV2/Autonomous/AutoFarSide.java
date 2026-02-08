package org.firstinspires.ftc.teamcode.RobotV2.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
public class AutoFarSide extends OpMode {

    //Data Classes
    private LimeLightVision limeLight;
    private RoadRunnerDataV2 rrData;

    private static boolean patternEnabled = true;
    private boolean isWaitTurret = false; //false
    private boolean forceFeedActive = true;

    @Override
    public void init(){

        RoadRunnerDataV2.isAutoPosStored = false;

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

        isWaitTurret = true;
        rrData.getRobotData().setStartFar(true);
        rrData.getRobotData().selectedSide();
        rrData.setDisplacement(RobotConstantsV2.FAR_BALL_DISTANCE);

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
                    Pose2d lineMidCollect = RobotConstantsV2.LINE_MID_COLLECT_BLUE;
                    Pose2d lineTop = RobotConstantsV2.LINE_TOP_PREP_BLUE;
                    Pose2d lineTopCollect = new Pose2d(-RobotConstantsV2.LINE_TOP_PREP_TRAVEL, -43, Math.toRadians(270));

                    Pose2d collectFromGateClose = RobotConstantsV2.COLLECT_GATE_CLOSE_BLUE;
                    Pose2d collectFromGateFar = RobotConstantsV2.COLLECT_GATE_FAR_BLUE;
                    Pose2d gate = RobotConstantsV2.GATE_OPEN_POSITION_BLUE;

                    //Far Side

                    rrData.setIntendedHeading(RobotConstantsV2.PINPOINT_HEADING_FAR_BLUE);

                    rrData.setBeginPose(RobotConstantsV2.BLUE_SPAWN_FAR);
                    rrData.createDrive();

                    //Common Positions
                    Pose2d spawn = rrData.getBeginPose();
                    Pose2d shootingPos = RobotConstantsV2.BLUE_SHOOT_FAR;

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

                    TrajectoryActionBuilder middleLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                            .setTangent(Math.toRadians(180))
                            .splineToLinearHeading(lineMid, Math.toRadians(270),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                    TrajectoryActionBuilder middleLineCollect = rrData.getDrive().actionBuilder(lineMid)
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(lineMid, Math.toRadians(90),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                    TrajectoryActionBuilder shootThird = rrData.getDrive().actionBuilder(lineMid)
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(shootingPos,Math.toRadians(0),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

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
                            shootFourth
                    });
                }

                //Trajectory Right
                else{
                    Pose2d lineBot = RobotConstantsV2.LINE_BOT_PREP_RED;
                    Pose2d lineBotCollect = RobotConstantsV2.LINE_BOT_COLLECT_RED;
                    Pose2d lineMid = RobotConstantsV2.LINE_MID_PREP_RED;
                    Pose2d lineMidCollect = RobotConstantsV2.LINE_MID_COLLECT_RED;
                    Pose2d lineTop = RobotConstantsV2.LINE_TOP_PREP_RED;
                    Pose2d lineTopCollect = new Pose2d(-RobotConstantsV2.LINE_TOP_PREP_TRAVEL, 43, Math.toRadians(90));;

                    Pose2d collectFromGateClose = RobotConstantsV2.COLLECT_GATE_CLOSE_RED;
                    Pose2d collectFromGateFar = RobotConstantsV2.COLLECT_GATE_FAR_RED;
                    Pose2d gate = RobotConstantsV2.GATE_OPEN_POSITION_RED;

                    rrData.setIntendedHeading(RobotConstantsV2.PINPOINT_HEADING_FAR_RED);

                    rrData.setBeginPose(RobotConstantsV2.RED_SPAWN_FAR);
                    rrData.createDrive();

                    //Common Positions
                    Pose2d spawn = rrData.getBeginPose();
                    Pose2d shootingPos = RobotConstantsV2.RED_SHOOT_FAR;

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

                    TrajectoryActionBuilder middleLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                            .setTangent(Math.toRadians(180))
                            .splineToLinearHeading(lineMid, Math.toRadians(90),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                    TrajectoryActionBuilder middleLineCollect = rrData.getDrive().actionBuilder(lineMid)
                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(lineMid, Math.toRadians(270),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                    TrajectoryActionBuilder shootThird = rrData.getDrive().actionBuilder(lineMid)
                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(shootingPos,Math.toRadians(0),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                    TrajectoryActionBuilder topLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                            .setTangent(Math.toRadians(150))
                            .splineToLinearHeading(lineTop, Math.toRadians(180),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                    TrajectoryActionBuilder topLineCollect = rrData.getDrive().actionBuilder(lineTop)
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(lineTopCollect, Math.toRadians(90),  new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                    TrajectoryActionBuilder shootFourth = rrData.getDrive().actionBuilder(lineTopCollect)
                            .setTangent(Math.toRadians(350))
                            .splineToLinearHeading(shootingPos,Math.toRadians(340),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                    TrajectoryActionBuilder spamGate = rrData.getDrive().actionBuilder(shootingPos)
                            .setTangent(Math.toRadians(180))
                            .splineToLinearHeading(lineMid, Math.toRadians(90),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED))

                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(gate, Math.toRadians(90), new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED))

                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(lineMid, Math.toRadians(270),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED))

                            .setTangent(Math.toRadians(270))
                            .splineToLinearHeading(shootingPos,Math.toRadians(0),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

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

        rrData.getRobotData().getTurret().toggleTurretFar(true);

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
                //new SleepAction(0.2),
                rrData.getTrajectory(4),
                rrData.intakeOff(),
                //rrData.getAlignTrajectoryAuto(limeLight.getFidYaw()),
                rrData.cycleQuickSlot(patternEnabled),
                rrData.waitForTurret(isWaitTurret),
                rrData.requestArtifactShots(patternEnabled),
                rrData.patternFireWaitSpeed(rrData.getDisplacement()),
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
                //rrData.getAlignTrajectoryAuto(limeLight.getFidYaw()),
                rrData.cycleQuickSlot(patternEnabled),
                rrData.waitForTurret(isWaitTurret),
                rrData.requestArtifactShots(patternEnabled),
                rrData.patternFireWaitSpeed(rrData.getDisplacement()),
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
                //rrData.getAlignTrajectoryAuto(limeLight.getFidYaw()),
                rrData.cycleQuickSlot(patternEnabled),
                rrData.waitForTurret(isWaitTurret),
                rrData.requestArtifactShots(patternEnabled),
                rrData.patternFireWaitSpeed(rrData.getDisplacement()),
                rrData.forceTransferDown(),
                rrData.cycleQuickSlot(patternEnabled)
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

                                rrData.requestArtifactShots(patternEnabled),
                                rrData.patternFireWaitSpeed(rrData.getDisplacement()),
                                rrData.forceTransferDown(),

                                buildBear[0],
                                buildBear[1],
                                buildBear[2],
                                buildBear[3],
//                                buildBear[4],
//                                buildBear[5],

                                rrData.setLooping(false),
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

        //RoadRunnerDataV2.lastAutoPosition = rrData.getDrive().localizer.getPose(); //Get last pos

        telemetry.addLine("Autonomous Completed!");
        telemetry.update();

        limeLight.killLimeLight();
    }
}