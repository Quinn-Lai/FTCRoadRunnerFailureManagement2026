package org.firstinspires.ftc.teamcode.RobotV2.TeleOp;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotV2.ClassData.LimeLightVision;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RoadRunnerDataV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotConstantsV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotDataV2;

@TeleOp
public class DriverControlBlue extends OpMode {

    //Data Classes
    private RoadRunnerDataV2 rrData;  //Road Runner Implementation
    private RobotDataV2 robotData;       //Basic Robot Mechanics
    private LimeLightVision limelight;
    private Action parkTemp;

    //Runs Once on Init
    @Override
    public void init(){

        robotData = new RobotDataV2(hardwareMap, telemetry);
        rrData = new RoadRunnerDataV2(robotData);
        rrData.createDashboard();
        limelight = new LimeLightVision(hardwareMap,telemetry,"blue");

        /** Road Runner Init */
        rrData.setBeginPoseFromAuto(); //TODO disablw
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

        if (limelight.getResults().isValid()){
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
    }

    //Loops after Start
    @Override
    public void loop(){

        limelight.updateOrientationIMU();

        /** First Updates */

        robotData.updateTelemetry(telemetry);
        robotData.getDriveTrain().setGamepad(gamepad1);

        /** Main Mode */
        switch (robotData.getDriveTrain().getCurrentMode()){
                //Auto Aim
                case ("auto"):

                    //Direction
                    robotData.getTurret().deactivateHumanIntakeMode();

                    //Passive Auto Aim
                    if (robotData.getTurret().isToggleTurretAim()){
                        if (limelight.getResults().isValid()){
                            robotData.getTurret().aimBall(limelight.getDisp());
                            robotData.updateTelemetry(telemetry);
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

                        robotData.getCarosel().autoIntakeCycle();
                        robotData.getCarosel().checkForAutoEject();

                        if (gamepad1.squareWasPressed() && !robotData.getCarosel().isEmptySpot()){
                            robotData.getCarosel().cycleCaroselManual();
                        }
                    }

                    robotData.getCarosel().updateIndicators(robotData.getDriveTrain().getCurrentMode(),limelight.getDisp(),limelight);

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
//                            RobotConstantsV2.updateCaroselPos((int)(gamepad1.touchpad_finger_1_x * RobotConstantsV2.caroselMultiplier));
//                            telemetry.addData("Increment:", (int)(gamepad1.touchpad_finger_1_x * RobotConstantsV2.caroselMultiplier));
//                            telemetry.addData("Position 1: ", RobotConstantsV2.caroselPos[0]);
                            RobotConstantsV2.CAROSEL_TOUCHPAD = (int)(gamepad1.touchpad_finger_1_x * RobotConstantsV2.caroselMultiplier);
                            robotData.getCarosel().cycleCarosel(robotData.getCarosel().getCurrentCycle());
                        }

                        if (gamepad1.squareWasPressed()){
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

                    //robotData.getCarosel().updateInventory();
                    if (robotData.getDriveTrain().getCurrentSubMode().equals("none")){

                        robotData.getCarosel().autoIntakeCycle();
                        //robotData.getCarosel().checkForAutoEject();

                        if (gamepad1.squareWasPressed() && !robotData.getCarosel().isEmptySpot()){
                            robotData.getCarosel().cycleCaroselManual();
                        }
                    }
                    robotData.getCarosel().updateIndicators(robotData.getDriveTrain().getCurrentMode(),0, limelight);
                    break;

                default:
                    telemetry.addLine("Richie is Bald");
                    break;
            }

        robotData.getCarosel().receiveAutoCycleStatus(); //TODO might be able to put here to make sure the call is always beign retreived

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

        //!rrData.getNotAutoPosStored()

        if (gamepad1.leftBumperWasPressed() && gamepad2.left_trigger > 0.5){

        }

        else if (gamepad1.right_trigger > 0.1){

            if (gamepad1.right_trigger < 0.5){
                rrData.requestPark(limelight.getAlliance());
            }

            else if (gamepad1.left_bumper && gamepad1.right_trigger >= 0.5){
                rrData.driveToPark(limelight.getAlliance());
            }


//            telemetry.addLine("poop");
//            parkTemp = rrData.getParkingTrajectory(limelight.getAlliance());
//            rrData.addTeleOpAction(parkTemp);
        }

//        telemetry.addData("Teleop:",rrData.getTele)
//        /** Auto Align */
////!rrData.getNotAutoPosStored()
//        //Deadman's switch
//        if (gamepad1.leftBumperWasPressed() && gamepad1.left_trigger > 0.5 && limelight.getResults().isValid()){ //TODO can switch getting current pos using limelight
//            rrData.killTeleOpActions(); //rrData.getDrive().getLocalizerPinpoint().getPose(),
//            rrData.addTeleOpAction(rrData.getAlignTrajectory(limelight.getFidYaw()));
//            rrData.runTeleOpActions();
//        }
//
//        else if (gamepad1.dpadDownWasPressed()){
//            rrData.killTeleOpActions();
//            rrData.addTeleOpAction(rrData.getParkingTrajectory(limelight.getAlliance())); //TODO test this
//        }
//
        rrData.updateTeleOpActions();


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

        if (limelight.getResults().isValid()){
            telemetry.addData("Current Pos: ", limelight.getCurrentPosLimelight());
            telemetry.addData("Current Yaw: ", limelight.getYaw());
        }

        //-----------------------------------------------------

        /** Sub Modes Requested */

        switch(robotData.getDriveTrain().getCurrentSubMode()){
            //Rapid Fire
            case ("rapidFire"): //TODO Not aligned with array in constants

                if (robotData.getCarosel().getRapidFireCurrentShotCount() >= RobotConstantsV2.RAPID_FIRE_MAX_SHOTS) {
                    robotData.getDriveTrain().endSubMode();
                    break;
                }

                switch (robotData.getCarosel().getCurrentSubModeQueue()){
                    case ("cycle"):

                        if (!robotData.getCarosel().isTransferCooldownActive()){
                            robotData.getCarosel().cycleRapidFire();

                            if (robotData.getCarosel().isCaroselInPlace()){
                                robotData.getCarosel().resetTransferStat();
                                robotData.getCarosel().resetShotSuccess();
                                robotData.getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
                                robotData.getCarosel().activateTransferInProg();
                            }
                        }

                        break;

                    case ("transfer"):
// || !robotData.getCarosel().detectedArtifact()

                        robotData.getCarosel().startTransferCooldown();
                        robotData.getCarosel().cycleTransferStartTimer();
                        robotData.getCarosel().checkShotSuccess();

                        if (robotData.getCarosel().getShotSuccessInstant() || robotData.getCarosel().isFailsafeSubmode()){
                            robotData.getCarosel().endFailsafeSubmode();
                            robotData.getCarosel().incrementRapidFireCurrentShotCount();
                            robotData.getCarosel().activateCycleInProg();
                            robotData.getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
                            break;
                        }

                        telemetry.addData("Shot Success", robotData.getCarosel().isShotSuccess());
                        telemetry.addData("Shot Success Instant", robotData.getCarosel().getShotSuccessInstant());

                        break;

                    default:
                        break;

                }

                break;

            case ("sortedFire"):

                if (robotData.getCarosel().getSortedFireCurrentShotCount() >= robotData.getCarosel().getMaxShots()) {
                    robotData.getDriveTrain().endSubMode();
                    break;
                }

                switch (robotData.getCarosel().getCurrentSubModeQueue()){
                    case ("cycle"):

                        if (!robotData.getCarosel().isTransferCooldownActive()){

                            robotData.getCarosel().cycleSortedFire(robotData.getCarosel().getSortedFireCurrentShotCount());

                            if (robotData.getCarosel().isCaroselInPlace()){
                                robotData.getCarosel().resetTransferStat();
                                robotData.getCarosel().resetShotSuccess();
                                robotData.getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
                                robotData.getCarosel().activateTransferInProg();
                            }

                        }

                        break;

                    case ("transfer"):

                        //|| !robotData.getCarosel().detectedArtifact()
                        robotData.getCarosel().startTransferCooldown();
                        robotData.getCarosel().cycleTransferStartTimer();
                        robotData.getCarosel().checkShotSuccess();

                        if (robotData.getCarosel().getShotSuccessInstant() || robotData.getCarosel().isFailsafeSubmode()){
                            robotData.getCarosel().endFailsafeSubmode();
                            robotData.getCarosel().incrementSortedFireCurrentShotCount();
                            robotData.getCarosel().activatePatternInProg();
                            robotData.getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
                            break;
                        }

                        telemetry.addData("Shot Success", robotData.getCarosel().isShotSuccess());
                        telemetry.addData("Shot Success Instant", robotData.getCarosel().getShotSuccessInstant());

                        break;

                    default:
                        break;

                }

                break;

            default:
                robotData.getCarosel().resetRapidFireCurrentShotCount();
                robotData.getCarosel().resetSortedFireCurrentShotCount();

                break;

        }

        robotData.getCarosel().transferReceiveTimer();

        /** Emergency Abort */
        if (gamepad1.dpadDownWasPressed()){
            robotData.getDriveTrain().endSubMode();
            //robotData.getCarosel().resetCarosel();
            RobotConstantsV2.CAROSEL_GLOBAL_INCREMENT = 0;
            RobotConstantsV2.CAROSEL_TOUCHPAD = 0;
        }

        //-----------------------------------------------------

        /** Last Calls */
        if (!(LimeLightVision.isFoundMotif)){
            LimeLightVision.failsafeMotif();
            limelight.updateMotifCode();        //Very smart ngl
        }

        robotData.getCarosel().updatePattern(LimeLightVision.motifCode); //TODO continueous updating not completely nessessary
        robotData.getDriveTrain().updateModeColor();
        robotData.getCarosel().cycleCarosel(robotData.getCarosel().getCurrentCycle());
        robotData.getDriveTrain().omniDrive();

        /** Telemetry */
        robotData.getDriveTrain().telemetryDriveTrain();
        robotData.getTurret().telemetryTurret(limelight.getDisp());
        robotData.getCarosel().telemetryCarosel();

        /** Rumbling */
        robotData.getDriveTrain().checkEndgame();

        /** AprilTag Seen */
        if (limelight.getResults().isValid()){
            telemetry.addLine("Eyes Open");
        }
        else{
            telemetry.addLine("Eyes Closed");
        }

        //telemetry.addLine(String.format("Positions (%f , %f , %f)", RobotConstantsV2.caroselPos[0], RobotConstantsV2.caroselPos[2], RobotConstantsV2.caroselPos[2]));
        telemetry.update();
    }

    //Runs at End
    @Override
    public void stop(){
        limelight.killLimeLight();
    }
}