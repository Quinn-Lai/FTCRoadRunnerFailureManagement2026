package org.firstinspires.ftc.teamcode.RobotV2.TeleOp;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotV2.ClassData.LimeLightVision;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RoadRunnerDataV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotConstantsV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotDataV2;

@TeleOp
public class DriverControlBlue extends OpMode {

    //Data Classes
    private RoadRunnerDataV2 rrData;       //Road Runner Implementation
    private RobotDataV2 robotData;         //Basic Robot Mechanics
    private LimeLightVision limelight;     //Lime Light Implementation

    private ElapsedTime testLoopTime;
    private double loopTime = 0;
    private boolean loopCheck = false;

    @Override
    public void init(){

        RobotConstantsV2.FAILSAFE_SUBMODE_TIMER = 200;
        RobotConstantsV2.COOLDOWN_SHOT = 200; //Transfer Shot
        RobotConstantsV2.COOLDOWN_PRE_SHOT = 0;

        robotData = new RobotDataV2(hardwareMap, telemetry);
        rrData = new RoadRunnerDataV2(robotData);
        rrData.createDashboard();
        limelight = new LimeLightVision(hardwareMap,telemetry,"blue");

        /** Road Runner Init */
        rrData.setBeginPoseFromAuto();
        rrData.createDrive();

        RobotConstantsV2.CAROSEL_TOUCHPAD = 0;

        limelight.setLimelightLocalizier(rrData.getDrive().getLocalizerPinpoint());

        limelight.initLimeLight();
        robotData.getDriveTrain().setGamepad(gamepad1);
        robotData.getCarosel().indicatorsInInit();
        rrData.updateRobotData(robotData);
    }

    @Override
    public void init_loop(){
        telemetry.addLine("Waiting for Round to Start");

        if (limelight.canSeeSomeAT()){

            limelight.updateMotifCode();
            limelight.telemetryLimeLight();
            robotData.getTurret().telemetryTurret(limelight.getDisp());
        }

        telemetry.update();
    }

    //Runs Once on Start
    @Override
    public void start(){
        robotData.getCarosel().forceTransferDown();
        rrData.updateRobotData(robotData);
        RobotDataV2.createRuntime();
        testLoopTime = new ElapsedTime();
    }

    //Loops after Start
    @Override
    public void loop(){

        if (gamepad2.leftBumperWasPressed()){
            testLoopTime.reset();
            loopCheck = true;
        }

        //robotData.getCarosel().updateInventory();

        /** Important First Updates */
        robotData.getDriveTrain().setGamepad(gamepad1);
        robotData.getCarosel().updateCaroselEncoder();
        limelight.updateOrientationIMU();
        robotData.updateTelemetry(telemetry);
        //robotData.getCarosel().removeArtifactPostShot();

        /** Main Mode */
        switch (robotData.getDriveTrain().getCurrentMode()){
                //Auto Aim
                case ("auto"):

                    //Direction
                    robotData.getTurret().deactivateHumanIntakeMode();

                    //Passive Auto Aim
                    if (robotData.getTurret().isToggleTurretAim()){
                        if (false){
                            robotData.getTurret().aimBall(limelight.getDisp());
                            robotData.getTurret().telemetryTurret(limelight.getDisp());
                        }
                        else{
                            robotData.getTurret().aimBall(RobotConstantsV2.CLOSE_BALL_DISTANCE);
                            telemetry.addLine("Eyes Closed");
                        }

                    }
                    else{
                        robotData.getTurret().killShooter();
                    }

                    if (gamepad1.dpadRightWasPressed()){
                        robotData.getDriveTrain().requestSortedFire();
                    }

                    //Auto Cycling (Don't want it cycling during submodes)
                    if (robotData.getDriveTrain().getCurrentSubMode().equals("none")){

                        robotData.getCarosel().checkForAutoEject();

                        //Auto cycling Not Active When the intake is on
                        if (robotData.getCarosel().isIntakeMotorOn()){
                            robotData.getCarosel().autoIntakeCycle();
                        }
                        else{
                            if (gamepad1.squareWasPressed()) robotData.getCarosel().cycleCaroselManual();
                        }
                    }

                    robotData.getCarosel().updateIndicators(robotData.getDriveTrain().getCurrentMode(),RobotConstantsV2.CLOSE_BALL_DISTANCE,limelight);

                    break;

                //Manual
                case("manual"):

                    //Direction
                    robotData.getTurret().deactivateHumanIntakeMode();

                    //Far Shot
                    if (gamepad1.optionsWasPressed()){
                        robotData.getTurret().switchTurretFar();
                    }

                    //Active Aim
                    if (robotData.getTurret().isToggleTurretAim()){
                        if (robotData.getTurret().isFarToggled()){
                            robotData.getTurret().aimBall(RobotConstantsV2.FAR_BALL_DISTANCE);
                        }
                        else{
                            robotData.getTurret().aimBall(RobotConstantsV2.CLOSE_BALL_DISTANCE);
                        }
                    }
                    else{
                        robotData.getTurret().killShooter();
                    }

                    //Cycling (Don't want cycling during submodes)
                    if (robotData.getDriveTrain().getCurrentSubMode().equals("none")){
                        if (gamepad1.touchpad){
                            RobotConstantsV2.CAROSEL_TOUCHPAD = (int)(gamepad1.touchpad_finger_1_x * RobotConstantsV2.caroselMultiplier);
                            robotData.getCarosel().cycleCarosel(robotData.getCarosel().getCurrentCycle());
                        }

                        else if (gamepad1.squareWasPressed()){
                            robotData.getCarosel().cycleCaroselManual();
                        }
                    }

                    robotData.getCarosel().updateIndicators(robotData.getDriveTrain().getCurrentMode(),RobotConstantsV2.FAR_BALL_DISTANCE,limelight);

                    break;

                case("humanIntake"):
                    if (robotData.getTurret().isToggleTurretAim()){
                        robotData.getTurret().activateHumanIntakeMode();
                    }
                    else{
                        robotData.getTurret().killShooter();
                        robotData.getTurret().deactivateHumanIntakeMode();
                    }

                    if (robotData.getDriveTrain().getCurrentSubMode().equals("none")){
                        if (gamepad1.squareWasPressed() && !robotData.getTurret().isToggleTurretAim()){
                            robotData.getCarosel().cycleCaroselManual();
                        }
                        else{
                            robotData.getCarosel().autoIntakeCycle();
                            //robotData.getCarosel().cycleCaroselManual();
                        }
                    }
                    robotData.getCarosel().updateIndicators(robotData.getDriveTrain().getCurrentMode(),0, limelight);
                    break;

                default:
                    telemetry.addLine("Richie is Bald");
                    break;
            }

        robotData.getCarosel().receiveAutoCycleStatus();

        //-----------------------------------------------------

        /** Shared Controls */

        /** Switch Turret Mode */
        if (gamepad1.circleWasPressed()){
            robotData.getTurret().switchTurretMode();
        }

        /** Mode Switching */
        if (gamepad1.shareWasPressed() || gamepad2.shareWasPressed()){
            robotData.getDriveTrain().switchMode();
        }

        //Can't switch to human player mode when in submode
        else if (gamepad1.dpadUpWasPressed() && robotData.getDriveTrain().getCurrentSubMode().equals("none")){
            robotData.getDriveTrain().switchHumanIntake();
        }

        /** Road Runner */

        //Auto Align
        if (gamepad1.left_trigger > RobotConstantsV2.TRIGGER_TOLERENCE){
            rrData.requestAlign(limelight.getFidYaw());
            rrData.runCurrentTeleOpAction();
        }

        //Auto Park
        else if (gamepad1.right_trigger > RobotConstantsV2.TRIGGER_TOLERENCE){
            rrData.requestPark(limelight.getAlliance());
            rrData.runCurrentTeleOpAction();
        }

        //Reset Start Up
        else{
            rrData.setTeleOpActionActive(false);
            robotData.getDriveTrain().omniDrive();
        }

//        if (gamepad1.left_bumper){
//            if (gamepad1.left_trigger > 0.1){
//                rrData.requestAlign();
//                rrData.addTeleopAction(rrData.getAlignTrajectory(limelight.getFidYaw()));
//            }
//            else if (gamepad1.right_trigger > 0.1){
//                rrData.runCurrentTeleOpAction();
//            }
//        }
//
//        //Auto Park
//        if (gamepad1.left_bumper){
//            if (gamepad1.left_trigger > 0.1){
//                rrData.requestPark(limelight.getAlliance());
//                rrData.addTeleopAction(rrData.getParkingTrajectory(limelight.getAlliance()));
//            }
//            else if (gamepad1.right_trigger > 0.1){
//                rrData.runCurrentTeleOpAction();
//            }
//        }
//
//        else{
//            robotData.getDriveTrain().omniDrive();
//        }


        /** Sub Modes */

        if (gamepad1.dpadLeftWasPressed()){
            robotData.getDriveTrain().requestRapidFire();
        }

        if (gamepad1.crossWasPressed()){
            robotData.getCarosel().switchIntake();
        }
        else if (gamepad1.triangleWasPressed()){
            robotData.getCarosel().switchReverseIntake();
        }

        /** Sub Modes */

        if (gamepad1.rightBumperWasPressed()){
            robotData.getCarosel().transferStartTimer();
        }

        //-----------------------------------------------------

        /** Sub Modes Requested */

//        switch(robotData.getDriveTrain().getCurrentSubMode()){
//            //Rapid Fire
//            case ("rapidFire"):
//
//                if (robotData.getCarosel().getRapidFireCurrentShotCount() >= RobotConstantsV2.RAPID_FIRE_MAX_SHOTS) {
//                    robotData.getDriveTrain().endSubMode();
//                    break;
//                }
//
//                switch (robotData.getCarosel().getCurrentSubModeQueue()){
//                    case ("cycle"):
//
//                        if (!robotData.getCarosel().isTransferCooldownActive()){
//                            robotData.getCarosel().cycleRapidFire();
//
//                            if (robotData.getCarosel().isCaroselInPlace()){
//                                robotData.getCarosel().resetShotSuccess();
//                                robotData.getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
//                                robotData.getCarosel().activateTransferInProg();
//                            }
//                        }
//
//                        break;
//
//                    case ("transfer"):
//
//                        if (!robotData.getCarosel().detectedArtifact() || robotData.getCarosel().getShotSuccessInstant() || robotData.getCarosel().isFailsafeSubmode()){
//                            robotData.getCarosel().endFailsafeSubmode();
//                            robotData.getCarosel().incrementRapidFireCurrentShotCount();
//                            robotData.getCarosel().activateCycleInProg();
//                            robotData.getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
//                            break;
//                        }
//
//                        robotData.getCarosel().subModeTransferStartTimer(); //Simplified
//
//                        robotData.getCarosel().checkShotSuccess();
//
//                        break;
//
//                    default:
//                        break;
//
//                }
//
//                break;
//
//            case ("sortedFire"):
//
//                if (robotData.getCarosel().getSortedFireCurrentShotCount() >= robotData.getCarosel().getMaxShots()) {
//                    robotData.getDriveTrain().endSubMode();
//                    break;
//                }
//
//                switch (robotData.getCarosel().getCurrentSubModeQueue()){
//                    case ("cycle"):
//
//                        if (!robotData.getCarosel().isTransferCooldownActive()){
//
//                            robotData.getCarosel().cycleSortedFire(robotData.getCarosel().getSortedFireCurrentShotCount());
//
//                            if (robotData.getCarosel().isCaroselInPlace()){
//                                robotData.getCarosel().resetShotSuccess();
//                                robotData.getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
//                                robotData.getCarosel().activateTransferInProg();
//                            }
//
//                        }
//
//                        break;
//
//                    case ("transfer"):
//
//                        if (robotData.getCarosel().getShotSuccessInstant() || robotData.getCarosel().isFailsafeSubmode()){
//                            robotData.getCarosel().endFailsafeSubmode();
//                            robotData.getCarosel().incrementSortedFireCurrentShotCount();
//                            robotData.getCarosel().activatePatternInProg();
//                            robotData.getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
//                            break;
//                        }
//
//                        robotData.getCarosel().subModeTransferStartTimer();
//
//                        robotData.getCarosel().checkShotSuccess();
//
//                        break;
//
//                    default:
//                        break;
//
//                }
//
//                break;
//
//            default:
//                robotData.getCarosel().resetRapidFireCurrentShotCount();
//                robotData.getCarosel().resetSortedFireCurrentShotCount();
//                break;
//
//        }

        robotData.getDriveTrain().executeSubMode();
        robotData.getCarosel().transferReceiveTimer();

        /** Emergency Abort */
        if (gamepad1.dpadDownWasPressed() || gamepad1.leftBumperWasPressed()){
            LimeLightVision.isFoundMotif = false;
            robotData.getDriveTrain().endSubMode();
            robotData.getCarosel().wipeInventory();
            //RobotConstantsV2.CAROSEL_GLOBAL_INCREMENT = 0;
        }

        //-----------------------------------------------------

        /** Last Calls */

        if (limelight.canSeeSomeAT()){
            telemetry.addData("Current Pos: ", limelight.getCurrentPosLimelight());
            telemetry.addData("Current Yaw: ", limelight.getYaw());
        }
        else{
            telemetry.addLine("Eyes Closed");
        }

        if (!(LimeLightVision.isFoundMotif)){
            LimeLightVision.failsafeMotif();
            limelight.updateMotifCode();
        }

        robotData.getCarosel().updatePattern(LimeLightVision.motifCode);
        robotData.getDriveTrain().updateModeColor();
        robotData.getCarosel().cycleCarosel(robotData.getCarosel().getCurrentCycle());

        /** Telemetry */
        robotData.getDriveTrain().telemetryDriveTrain();
        robotData.getCarosel().telemetryCarosel();

        switch (robotData.getDriveTrain().getCurrentMode()){
            //Auto Aim
            case ("auto"):

                robotData.getTurret().telemetryTurret(limelight.getDisp());

                break;

            //Manual
            case("manual"):

                if (robotData.getTurret().isFarToggled()){
                    robotData.getTurret().telemetryTurret(RobotConstantsV2.FAR_BALL_DISTANCE);
                }

                else{
                    robotData.getTurret().telemetryTurret(RobotConstantsV2.CLOSE_BALL_DISTANCE);
                }

                break;

            case("humanIntake"):

                telemetry.addLine("Human Intake Mode Is Active!");

                break;

            default:
                telemetry.addLine("Richie is Bald");
                break;
        }

        /** Rumbling */
        robotData.getDriveTrain().checkEndgame();

        if (loopCheck){
            loopTime = testLoopTime.milliseconds();
            loopCheck = false;
        }

        telemetry.addData("Last Loop Interval: ", loopTime);

        telemetry.update();
    }

    //Runs at End
    @Override
    public void stop(){
        limelight.killLimeLight();
    }
}