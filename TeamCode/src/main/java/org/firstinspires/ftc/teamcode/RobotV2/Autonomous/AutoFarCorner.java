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
public class AutoFarCorner extends OpMode {

    //Data Classes
    private LimeLightVision limeLight;
    private RoadRunnerDataV2 rrData;

    private static boolean patternEnabled = true;
    private boolean isWaitTurret = false; //false
    private boolean forceFeedActive = true;

    @Override
    public void init(){

        RoadRunnerDataV2.isAutoPosStored = false;

        rrData = new RoadRunnerDataV2(new RobotDataV2(hardwareMap, telemetry));
        limeLight = new LimeLightVision(hardwareMap,telemetry,"blue");

        RobotConstantsV2.AUTO_FAILSAFE_TIMER = RobotConstantsV2.AUTO_FAILSAFE_TIMER_FAR;

        rrData.createDashboard();

        limeLight.updateAtHeight(rrData.getRobotData().getTurret().getHeightOfLauncher());
        limeLight.initLimeLight();

        LimeLightVision.isFoundMotif = false;

        rrData.getRobotData().getCarosel().forceTransferDown();
        rrData.getRobotData().getCarosel().setInventoryAuto();
        rrData.getRobotData().getCarosel().indicatorsInInit();
        rrData.getRobotData().getCarosel().cycleOrigin();
        rrData.getRobotData().getCarosel().updateCaroselEncoder();

        isWaitTurret = true;
        rrData.getRobotData().setStartFar(true);
        rrData.getRobotData().selectedSide();
        rrData.setDisplacement(RobotConstantsV2.FAR_BALL_DISTANCE);

        RobotConstantsV2.CAROSEL_DETECTED_ARTIFACT_DELAY = RobotConstantsV2.CAROSEL_DETECTED_ARTIFACT_DELAY_AUTO;
    }

    @Override
    public void init_loop(){

        //Color Selection & OpenCV

        //limeLight.updateOrientationIMU(rrData.getYaw());
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

                    //Each Route

                    TrajectoryActionBuilder shootFirst = rrData.getDrive().actionBuilder(spawn)
                            .setTangent(Math.toRadians(180))
                            .splineToLinearHeading(shootingPos,Math.toRadians(180));

                    TrajectoryActionBuilder collectCorner = rrData.getDrive().actionBuilder(shootingPos)
                            //.turn(Math.toRadians(30))
                            .setTangent(Math.toRadians(270))
                            .splineToSplineHeading(RobotConstantsV2.CORNER_BLUE,Math.toRadians(270), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED))
                            .setTangent(Math.toRadians(135))
                            .splineToSplineHeading(RobotConstantsV2.CORNER_BLUE_SHIMMY,Math.toRadians(225),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED))
                            .setTangent(Math.toRadians(45))
                            .splineToSplineHeading(RobotConstantsV2.CORNER_BLUE,Math.toRadians(315), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

                    TrajectoryActionBuilder returnShoot = collectCorner.fresh()
                            .setTangent(Math.toRadians(135))
                            .splineToLinearHeading(shootingPos,Math.toRadians(145), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                    TrajectoryActionBuilder park = rrData.getDrive().actionBuilder(shootingPos)
                            .setTangent(Math.toRadians(180))
                            .lineToX(-40);


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
                            returnShoot,

                            collectCorner,
                            returnShoot,

                            park
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

                    //Each Route

                    TrajectoryActionBuilder shootFirst = rrData.getDrive().actionBuilder(spawn)
                            .setTangent(Math.toRadians(180))
                            .splineToLinearHeading(shootingPos,Math.toRadians(180));

                    TrajectoryActionBuilder collectCorner = rrData.getDrive().actionBuilder(shootingPos)
                            //.turn(Math.toRadians(-30))
                            .setTangent(Math.toRadians(90))
                            .splineToSplineHeading(RobotConstantsV2.CORNER_RED,Math.toRadians(90), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED))
                            .setTangent(Math.toRadians(225))
                            .splineToSplineHeading(RobotConstantsV2.CORNER_RED_SHIMMY,Math.toRadians(135),new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED))
                            .setTangent(Math.toRadians(315))
                            .splineToSplineHeading(RobotConstantsV2.CORNER_RED,Math.toRadians(45), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED));

                    TrajectoryActionBuilder returnShoot = collectCorner.fresh()
                            .setTangent(Math.toRadians(225))
                            .splineToLinearHeading(shootingPos,Math.toRadians(315), new TranslationalVelConstraint(RobotConstantsV2.AUTO_FAST_SPEED));

                    TrajectoryActionBuilder park = rrData.getDrive().actionBuilder(shootingPos)
                            .setTangent(Math.toRadians(180))
                            .lineToX(40);

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
                            returnShoot,

                            collectCorner,
                            returnShoot,

                            park

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
                                rrData.shootArtifacts(rrData.getDisplacement(), patternEnabled),
                                rrData.forceTransferDown(),
                                rrData.cycleFirstSlot(),

                                //Second Set
                                getFarPath(2,3),

                                //Third Set
                                getFarPath(4,5),

                                rrData.intakeOn(),
                                rrData.startFailsafeTimer(),

                                new ParallelAction(
                                        rrData.getTrajectory(6),
                                        rrData.checkAutoIntake()
                                ),


//
//                                //Fourth Set
//                                getFarPath(6,7),
//
//                                //Fifth Set
//                                getFarPath(8,9),
//
//                                //Sixth Set
//                                getFarPath(10,11),

                                //Park
                                //rrData.getTrajectory(12),

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

        if (rrData.getDrive() != null){
            RoadRunnerDataV2.isAutoPosStored = true;
            RoadRunnerDataV2.lastAutoPosition = rrData.getDrive().localizer.getPose();
        }

        rrData.setLooping(false);

        //RoadRunnerDataV2.lastAutoPosition = rrData.getDrive().localizer.getPose(); //Get last pos

        telemetry.addLine("Autonomous Completed!");
        telemetry.update();

        limeLight.killLimeLight();
    }

    private SequentialAction getFarPath(int go, int back){
        return new SequentialAction(
                rrData.intakeOn(),
                rrData.startFailsafeTimer(),

                new ParallelAction(
                        rrData.getTrajectory(go),
                        rrData.checkAutoIntake()
                ),

                rrData.intakeReverse(),
                rrData.cycleQuickSlot(patternEnabled),

                rrData.getTrajectory(back),
                rrData.intakeOff(),

                //rrData.waitForTurret(isWaitTurret),
                rrData.requestArtifactShots(patternEnabled),
                rrData.shootArtifacts(rrData.getDisplacement(), patternEnabled),
                rrData.forceTransferDown(),
                rrData.cycleFirstSlot()
        );
    }


}