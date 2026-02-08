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
public class AutoFarCorner extends OpMode {

    //Data Classes
    private LimeLightVision limeLight;
    private RoadRunnerDataV2 rrData;

    private static boolean patternEnabled = true;
    private boolean isWaitTurret = false; //false
    private boolean forceFeedActive = true;

    @Override
    public void init(){

        RoadRunnerDataV2.isAutoPosStored = false; //TODO rememeber

        RobotConstantsV2.FAILSAFE_SUBMODE_TIMER = RobotConstantsV2.FAILSAFE_SUBMODE_TIMER_AUTO;
        RobotConstantsV2.COOLDOWN_SHOT = RobotConstantsV2.COOLDOWN_SHOT_AUTO;
        RobotConstantsV2.COOLDOWN_PRE_SHOT = RobotConstantsV2.COOLDOWN_PRE_SHOT_AUTO;

        rrData = new RoadRunnerDataV2(new RobotDataV2(hardwareMap, telemetry));
        limeLight = new LimeLightVision(hardwareMap,telemetry,"blue");

        rrData.createDashboard();

        limeLight.updateAtHeight(rrData.getRobotData().getTurret().getHeightOfLauncher());
        limeLight.initLimeLight();

        LimeLightVision.isFoundMotif = false;
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

                    rrData.setIntendedHeading(RobotConstantsV2.PINPOINT_HEADING_FAR_BLUE);
                    rrData.setBeginPose(RobotConstantsV2.BLUE_SPAWN_FAR);
                    rrData.createDrive();

                    Pose2d spawn = rrData.getBeginPose();
                    Pose2d shootingPos = RobotConstantsV2.BLUE_SHOOT_FAR;
                    Pose2d corner = RobotConstantsV2.CORNER_BLUE;

                    //Each Route

                    TrajectoryActionBuilder shootFirst = rrData.getDrive().actionBuilder(spawn)
                            .setTangent(Math.toRadians(180))
                            .splineToLinearHeading(shootingPos,Math.toRadians(180));

                    TrajectoryActionBuilder collectCorner = rrData.getDrive().actionBuilder(shootingPos)
                            .setTangent(Math.toRadians(225))
                            .splineToLinearHeading(corner,Math.toRadians(315));

                    TrajectoryActionBuilder returnShoot = rrData.getDrive().actionBuilder(corner)
                            .setTangent(Math.toRadians(135))
                            .splineToLinearHeading(shootingPos,Math.toRadians(145));


                    //Insert Trajectory Paths Here
                    rrData.createTrajectoryPath(new TrajectoryActionBuilder[]{
                            shootFirst,
                            collectCorner,
                            returnShoot,

                            collectCorner,
                            returnShoot,

                            collectCorner,
                            returnShoot,

                            collectCorner,
                            returnShoot
                    });
                }

                //Trajectory Right
                else{

                    rrData.setIntendedHeading(RobotConstantsV2.PINPOINT_HEADING_FAR_RED);
                    rrData.setBeginPose(RobotConstantsV2.RED_SPAWN_FAR);
                    rrData.createDrive();

                    //Common Positions
                    Pose2d spawn = rrData.getBeginPose();
                    Pose2d shootingPos = RobotConstantsV2.RED_SHOOT_FAR;
                    Pose2d corner = RobotConstantsV2.CORNER_RED;

                    //Each Route

                    TrajectoryActionBuilder shootFirst = rrData.getDrive().actionBuilder(spawn)
                            .setTangent(Math.toRadians(180))
                            .splineToLinearHeading(shootingPos,Math.toRadians(180));

                    TrajectoryActionBuilder collectCorner = rrData.getDrive().actionBuilder(shootingPos)
                            .setTangent(Math.toRadians(135))
                            .splineToLinearHeading(corner,Math.toRadians(45));

                    TrajectoryActionBuilder returnShoot = rrData.getDrive().actionBuilder(corner)
                            .setTangent(Math.toRadians(225))
                            .splineToLinearHeading(shootingPos,Math.toRadians(315));

                    //Insert Trajectory Paths Here
                    rrData.createTrajectoryPath(new TrajectoryActionBuilder[]{
                            shootFirst,
                            collectCorner,
                            returnShoot,

                            collectCorner,
                            returnShoot,

                            collectCorner,
                            returnShoot,

                            collectCorner,
                            returnShoot

                    });

                }

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
        rrData.setLoopStatus(true);

        Actions.runBlocking(
                new ParallelAction(

                        //Repeated Stuff

                        rrData.indicatorsUpdate(limeLight),
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


                                //Second Ball (Intake + Shoot)
                                rrData.intakeOn(),
                                rrData.startFailsafeTimer(),

                                new ParallelAction(
                                        rrData.getTrajectory(2),
                                        rrData.checkAutoIntake()
                                ),

                                rrData.intakeReverse(),
                                rrData.cycleQuickSlot(patternEnabled),

                                rrData.getTrajectory(3),
                                rrData.intakeOff(),

                                //rrData.waitForTurret(isWaitTurret),
                                rrData.requestArtifactShots(patternEnabled),
                                rrData.patternFireWaitSpeed(rrData.getDisplacement()),
                                rrData.forceTransferDown(),

                                //Third Ball (Intake + Shoot)
                                rrData.intakeOn(),
                                rrData.startFailsafeTimer(),

                                new ParallelAction(
                                        rrData.getTrajectory(4),
                                        rrData.checkAutoIntake()
                                ),

                                rrData.intakeReverse(),
                                rrData.cycleQuickSlot(patternEnabled),

                                rrData.getTrajectory(5),
                                rrData.intakeOff(),

                                //rrData.waitForTurret(isWaitTurret),
                                rrData.requestArtifactShots(patternEnabled),
                                rrData.patternFireWaitSpeed(rrData.getDisplacement()),
                                rrData.forceTransferDown(),

                                //Fourth Ball (Intake + Shoot)
                                rrData.intakeOn(),
                                rrData.startFailsafeTimer(),

                                new ParallelAction(
                                        rrData.getTrajectory(6),
                                        rrData.checkAutoIntake()
                                ),

                                rrData.intakeReverse(),
                                rrData.cycleQuickSlot(patternEnabled),

                                rrData.getTrajectory(7),
                                rrData.intakeOff(),

                                //rrData.waitForTurret(isWaitTurret),
                                rrData.requestArtifactShots(patternEnabled),
                                rrData.patternFireWaitSpeed(rrData.getDisplacement()),
                                rrData.forceTransferDown(),


                                //Fifth Ball (Intake + Shoot)
                                rrData.intakeOn(),
                                rrData.startFailsafeTimer(),

                                new ParallelAction(
                                        rrData.getTrajectory(8),
                                        rrData.checkAutoIntake()
                                ),

                                rrData.intakeReverse(),
                                rrData.cycleQuickSlot(patternEnabled),

                                rrData.getTrajectory(9),
                                rrData.intakeOff(),

                                //rrData.waitForTurret(isWaitTurret),
                                rrData.requestArtifactShots(patternEnabled),
                                rrData.patternFireWaitSpeed(rrData.getDisplacement()),
                                rrData.forceTransferDown(),

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