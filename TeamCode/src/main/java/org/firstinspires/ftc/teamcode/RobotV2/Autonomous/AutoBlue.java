package org.firstinspires.ftc.teamcode.RobotV2.Autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotV2.ClassData.AprilTagVisionV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.LimeLightVision;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RoadRunnerDataV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotConstantsV2;
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
        robotData.getCarosel().setInventoryAuto();
        robotData.getCarosel().resetCarosel();
    }

    @Override
    public void init_loop(){

        //Color Selection & OpenCV

        rrData.updateRobotData(robotData);

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

                    Pose2d lineBot = new Pose2d(34, -28, Math.toRadians(270));
                    Pose2d lineBotCollect = new Pose2d(34, -52, Math.toRadians(270));
                    Pose2d lineMid = new Pose2d(11, -28, Math.toRadians(270));
                    Pose2d lineMidCollect = new Pose2d(11, -52, Math.toRadians(270));
                    Pose2d lineTop = new Pose2d(-12, -28, Math.toRadians(270));
                    Pose2d lineTopCollect = new Pose2d(-12, -49, Math.toRadians(270));

                    //Far Side
                    if (robotData.isStartFar()){
                        rrData.setBeginPose(new Pose2d(61.065, -13, Math.toRadians(180)));
                        rrData.createDrive();

                        //Common Positions
                        Pose2d spawn = rrData.getBeginPose();
                        Pose2d shootingPos = new Pose2d(51, -13, Math.toRadians(200));

                        //Each Route

                        //Possibly just move this to rrData Class

                        TrajectoryActionBuilder shootFirst = rrData.getDrive().actionBuilder(spawn)
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(shootingPos,Math.toRadians(180));

                        TrajectoryActionBuilder bottomLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(200))
                                .splineToLinearHeading(lineBot, Math.toRadians(270));

                        TrajectoryActionBuilder bottomLineCollect = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(lineBotCollect, Math.toRadians(270), new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                        TrajectoryActionBuilder shootSecond = rrData.getDrive().actionBuilder(lineBotCollect)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(shootingPos,Math.toRadians(20));

                        TrajectoryActionBuilder middleLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(200))
                                .splineToLinearHeading(lineMid, Math.toRadians(180));

                        TrajectoryActionBuilder middleLineCollect = rrData.getDrive().actionBuilder(lineMid)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(lineMidCollect, Math.toRadians(270), new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                        TrajectoryActionBuilder shootThird = rrData.getDrive().actionBuilder(lineMidCollect)
                                .setTangent(Math.toRadians(45))
                                .splineToLinearHeading(shootingPos,Math.toRadians(20));

                        TrajectoryActionBuilder topLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(200))
                                .splineToLinearHeading(lineTop, Math.toRadians(180));

                        TrajectoryActionBuilder topLineCollect = rrData.getDrive().actionBuilder(lineTop)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(lineTopCollect, Math.toRadians(270), new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                        TrajectoryActionBuilder shootFourth = rrData.getDrive().actionBuilder(lineTopCollect)
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

                    //Close Side
                    else{
                        rrData.setBeginPose(new Pose2d(-52, -48.5, Math.toRadians(54.046)));
                        rrData.createDrive();

                        //Common Positions
                        Pose2d spawn = rrData.getBeginPose();
                        Pose2d shootingPos = new Pose2d(-15, -15, Math.toRadians(225));

                        //Each Route

                        TrajectoryActionBuilder shootFirst = rrData.getDrive().actionBuilder(spawn)
                                .setTangent(Math.toRadians(30))
                                .splineToLinearHeading(shootingPos,Math.toRadians(30));

                        TrajectoryActionBuilder topLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(315))
                                .splineToLinearHeading(lineTop, Math.toRadians(270));


                        TrajectoryActionBuilder topLineCollect = rrData.getDrive().actionBuilder(lineTop)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(lineTopCollect,Math.toRadians(270),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));


                        TrajectoryActionBuilder shootSecond = rrData.getDrive().actionBuilder(lineTopCollect)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(shootingPos,Math.toRadians(90));

                        TrajectoryActionBuilder middleLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(lineMid, Math.toRadians(270));

                        TrajectoryActionBuilder middleLineCollect = rrData.getDrive().actionBuilder(lineMid)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(lineMidCollect,Math.toRadians(270),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                        TrajectoryActionBuilder shootThird = rrData.getDrive().actionBuilder(lineMidCollect)
                                .setTangent(Math.toRadians(135))
                                .splineToLinearHeading(shootingPos,Math.toRadians(90));

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
                    Pose2d lineBot = new Pose2d(34, 28, Math.toRadians(90));
                    Pose2d lineBotCollect = new Pose2d(34, 52, Math.toRadians(90));
                    Pose2d lineMid = new Pose2d(11, 28, Math.toRadians(90));
                    Pose2d lineMidCollect = new Pose2d(11, 52, Math.toRadians(90));
                    Pose2d lineTop = new Pose2d(-12, 28, Math.toRadians(90));
                    Pose2d lineTopCollect = new Pose2d(-12, 49, Math.toRadians(90));

                    if (robotData.isStartFar()){
                        rrData.setBeginPose( new Pose2d(61.065, 13, Math.toRadians(180)));
                        rrData.createDrive();

                        //Common Positions
                        Pose2d spawn = rrData.getBeginPose();
                        Pose2d shootingPos = new Pose2d(51, 13, Math.toRadians(160));

                        //Each Route

                        //Possibly just move this to rrData Class

                        TrajectoryActionBuilder shootFirst = rrData.getDrive().actionBuilder(spawn)
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(shootingPos,Math.toRadians(180));

                        TrajectoryActionBuilder bottomLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(150))
                                .splineToSplineHeading(lineBot, Math.toRadians(90));

                        TrajectoryActionBuilder bottomLineCollect = rrData.getDrive().actionBuilder(lineBot)
                                .setTangent(Math.toRadians(90))
                                .splineToSplineHeading(lineBotCollect, Math.toRadians(90), new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                        TrajectoryActionBuilder shootSecond = rrData.getDrive().actionBuilder(lineBotCollect)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(shootingPos,Math.toRadians(340));

                        TrajectoryActionBuilder middleLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(150))
                                .splineToLinearHeading(lineMid, Math.toRadians(180));

                        TrajectoryActionBuilder middleLineCollect = rrData.getDrive().actionBuilder(lineMid)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(lineMidCollect, Math.toRadians(90), new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                        TrajectoryActionBuilder shootThird = rrData.getDrive().actionBuilder(lineMidCollect)
                                .setTangent(Math.toRadians(315))
                                .splineToLinearHeading(shootingPos,Math.toRadians(315));

                        TrajectoryActionBuilder topLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(150))
                                .splineToLinearHeading(lineTop, Math.toRadians(180));

                        TrajectoryActionBuilder topLineCollect = rrData.getDrive().actionBuilder(lineTop)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(lineTopCollect, Math.toRadians(90),  new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                        TrajectoryActionBuilder shootFourth = rrData.getDrive().actionBuilder(lineTopCollect)
                                .setTangent(Math.toRadians(350))
                                .splineToLinearHeading(shootingPos,Math.toRadians(340));

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

                        rrData.setBeginPose(new Pose2d(-52, 48.5, Math.toRadians(305.954)));
                        rrData.createDrive();

                        //Common Positions
                        Pose2d spawn = rrData.getBeginPose();
                        Pose2d shootingPos = new Pose2d(-15, 15, Math.toRadians(135));

                        //Each Route

                        TrajectoryActionBuilder shootFirst = rrData.getDrive().actionBuilder(spawn)
                                .setTangent(Math.toRadians(330))
                                .splineToLinearHeading(shootingPos,Math.toRadians(330));

                        TrajectoryActionBuilder topLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(45))
                                .splineToLinearHeading(lineTop, Math.toRadians(90));


                        TrajectoryActionBuilder topLineCollect = rrData.getDrive().actionBuilder(lineTop)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(lineTopCollect,Math.toRadians(90),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));


                        TrajectoryActionBuilder shootSecond = rrData.getDrive().actionBuilder(lineTopCollect)
                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(shootingPos,Math.toRadians(270));

                        TrajectoryActionBuilder middleLinePrep = rrData.getDrive().actionBuilder(shootingPos)
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(lineMid, Math.toRadians(90));

                        TrajectoryActionBuilder middleLineCollect = rrData.getDrive().actionBuilder(lineMid)
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(lineMidCollect,Math.toRadians(90),new TranslationalVelConstraint(RobotConstantsV2.AUTO_SLOW_SPEED));

                        TrajectoryActionBuilder shootThird = rrData.getDrive().actionBuilder(lineMidCollect)
                                .setTangent(Math.toRadians(225))
                                .splineToLinearHeading(shootingPos,Math.toRadians(270));

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
                robotData.getCarosel().indicatorsInInit();
            }

            telemetry.addLine("Completed Initialization..... \n --------------------------" );
            telemetry.addData("Found Motif", limeLight.foundMotif());
            if (limeLight.foundMotif()){
                telemetry.addData("Motif Code: ", LimeLightVision.motifCode[0] + ", " + LimeLightVision.motifCode[1] + ", " + LimeLightVision.motifCode[2]);
            }
            telemetry.addData("Pinpoint Yaw: ", rrData.getDrive().getLocalizerPinpoint().getHeadingLocalizerDegrees());
            telemetry.addData("Pinpoint Heading (diff): ", rrData.getDrive().getLocalizerPinpoint().getPose().heading);
            telemetry.addData("Pinpoint x: ", rrData.getDrive().getLocalizerPinpoint().getPose().position.x);
            telemetry.addLine(String.format("Inventory: (%s, %s, %s)", robotData.getCarosel().getInventory()[0], robotData.getCarosel().getInventory()[1], robotData.getCarosel().getInventory()[2]));
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

        //Far side Auto

        if (robotData.isStartFar()){
            telemetry.addLine("Beginning Auto: Far Side");
            telemetry.update();

            rrData.getRobotData().getTurret().toggleTurretFar(true);

        }

        //Close Side Auto
        else{
            telemetry.addLine("Beginning Auto: Close Side");

            rrData.getRobotData().getTurret().toggleTurretFar(false);

            telemetry.update();
        }

        rrData.getRobotData().getCarosel().updatePattern(LimeLightVision.motifCode);

        Actions.runBlocking(
                new SequentialAction(

                        rrData.setIsDoneInit(false),
                        new ParallelAction(
                                rrData.turretPIDSetUp(),
                                rrData.getTrajectory(1),
                                rrData.indicatorsUpdate(limeLight),
                                rrData.telemetryAuto(),
                                rrData.locateAprilTag(limeLight)
                        ),

                        rrData.setIsDoneInit(false),
//                            new ParallelAction(
//                                    rrData.indicatorsUpdate(limeLight),
//                                    rrData.turretPIDOn(),
//                                    rrData.telemetryAuto(),
//                                    new SequentialAction(
//                                            rrData.requestPatternFire(),
//                                            rrData.patternFire()
//                                    )
//                            )

                        new ParallelAction(

                                //Shoot First

                                rrData.indicatorsUpdate(limeLight),
                                rrData.telemetryAuto(),
                                rrData.turretPIDOn(),

                                new SequentialAction(

                                        //First Ball

                                        rrData.requestPatternFire(),
                                        rrData.patternFire(),
                                        //rrData.setTurretDone(true),
                                        rrData.forceTransferDown(),

                                        //Collect Bottom Line
                                        rrData.getTrajectory(2),
                                        rrData.intakeOn(),
                                        rrData.startFailsafeTimer(),
                                        new ParallelAction(
                                                rrData.getTrajectory(3),
                                                rrData.checkAutoIntake()
                                        ),
                                        rrData.intakeOff(),

                                        //Second Shot

                                        rrData.getTrajectory(4),
                                        rrData.requestPatternFire(),
                                        rrData.patternFire(),
                                        //rrData.setTurretDone(true),
                                        rrData.forceTransferDown(),

                                        //Collect Middle Line
                                        rrData.getTrajectory(5),
                                        rrData.intakeOn(),
                                        rrData.startFailsafeTimer(),
                                        new ParallelAction(
                                                rrData.getTrajectory(6),
                                                rrData.checkAutoIntake()
                                        ),
                                        rrData.intakeOff(),

                                        //Third Shot

                                        rrData.getTrajectory(7),
                                        rrData.requestPatternFire(),
                                        rrData.patternFire(),
                                        //rrData.setTurretDone(true),
                                        rrData.forceTransferDown(),

                                        //Collect Top Line
                                        rrData.getTrajectory(8),
                                        rrData.intakeOn(),
                                        rrData.startFailsafeTimer(),
                                        new ParallelAction(
                                                rrData.getTrajectory(9),
                                                rrData.checkAutoIntake()
                                        ),
                                        rrData.intakeOff(),
                                        rrData.setTurretDone(true),
                                        rrData.setIsDoneInit(false)
                                )
                        ),

                        rrData.killTurret()
                )
        );



    }

    @Override
    public void loop(){

    }

    @Override
    public void stop() {

        //RoadRunnerDataV2.lastAutoPosition = rrData.getDrive().getLocalizerPinpoint().getPose(); //Get last pos

        telemetry.addLine("Autonomous Completed!");
        telemetry.addData("Time Spent: ", RobotDataV2.getRuntime());
        limeLight.killLimeLight();
    }
}

//                            rrData.cyclePattern(0),
//                            new SleepAction(1),
//                            rrData.switchTransfer(),
//                            new SleepAction(1),
//                            rrData.switchTransfer(),
//                            new SleepAction(1),
//                            rrData.cyclePattern(1),
//                            new SleepAction(1),
//                            rrData.switchTransfer(),
//                            new SleepAction(1),
//                            rrData.switchTransfer(),
//                            rrData.cyclePattern(2),
//                            new SleepAction(1),
//                            rrData.switchTransfer(),
//                            new SleepAction(1),
//                            rrData.switchTransfer()