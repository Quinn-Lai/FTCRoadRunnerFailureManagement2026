package org.firstinspires.ftc.teamcode.RobotV2.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotV2.ClassData.AprilTagVisionV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.LimeLightVision;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RoadRunnerDataV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotConstantsV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotDataV2;

import java.util.Arrays;

@TeleOp
public class DriverControlBlue extends OpMode {

    //Data Classes
    private RoadRunnerDataV2 rrData;  //Road Runner Implementation
    private RobotDataV2 robotData;       //Basic Robot Mechanics
    private LimeLightVision limelight;

    //Runs Once on Init
    @Override
    public void init(){
        RobotConstantsV2.updateCaroselPos(0);

        limelight = new LimeLightVision(hardwareMap,telemetry,"blue");
        robotData = new RobotDataV2(hardwareMap, telemetry);
        rrData = new RoadRunnerDataV2(robotData);

        limelight.initLimeLight();
        robotData.getDriveTrain().setGamepad(gamepad1);
        robotData.getCarosel().indicatorsInInit();
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

        //TODO REMOVE THIS IT IS TEMP
        //LimeLightVision.isFoundMotif = false;

        robotData.getCarosel().forceTransferDown();
        RobotDataV2.createRuntime();
        //rrData.createDashboard();
    }

    //Loops after Start
    @Override
    public void loop(){

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
                    if (robotData.getDriveTrain().getCurrentSubMode().equals("none")) robotData.getCarosel().autoIntakeCycle();

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
                            RobotConstantsV2.updateCaroselPos((int)(gamepad1.touchpad_finger_1_x * RobotConstantsV2.caroselMultiplier));
                            telemetry.addData("Increment:", (int)(gamepad1.touchpad_finger_1_x * RobotConstantsV2.caroselMultiplier));
                            telemetry.addData("Position 1: ", RobotConstantsV2.caroselPos[0]);
                            robotData.getCarosel().cycleCarosel(robotData.getCarosel().getCurrentCycle());
                        }

                        if (gamepad1.squareWasPressed()){
                            robotData.getCarosel().cycleCaroselManual();
                        }
                    }

                    robotData.getCarosel().updateIndicators(robotData.getDriveTrain().getCurrentMode(),0,limelight);

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
                    if (robotData.getDriveTrain().getCurrentSubMode().equals("none")) robotData.getCarosel().autoIntakeCycle();

                    robotData.getCarosel().updateIndicators(robotData.getDriveTrain().getCurrentMode(),0, limelight);
                    break;

                default:
                    telemetry.addLine("Richie is Bald");
                    break;
            }

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

        /** Auto Park */

        if (gamepad1.left_trigger > 0.75 && gamepad1.right_trigger > 0.75){
            rrData.addTeleOpAction(rrData.getTestSeqAction()); //Replace with Auto Park
            rrData.runTeleOpActions();
        }

        /** Sub Modes */

        if (gamepad1.dpadLeftWasPressed()){
            robotData.getDriveTrain().requestRapidFire();
        }

        if (gamepad1.crossWasPressed()){
            robotData.getCarosel().switchIntake();
        }

        /** Sub Modes */

        if (gamepad1.rightBumperWasPressed()){
            robotData.getCarosel().transferStartTimer();
        }

        /** Auto Align */

        //Deadman's switch
        if (gamepad1.leftBumperWasPressed() && gamepad1.left_trigger > 0.5){ //TODO use limelight to get 2D pose, generate a trajectory, run it
            rrData.addTeleOpAction(rrData.getTestSeqAction()); //Replace with Auto Park
            rrData.runTeleOpActions();
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
                                robotData.getCarosel().resetShotSuccess();
                                robotData.getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
                                robotData.getCarosel().activateTransferInProg();
                            }
                        }

                        break;

                    case ("transfer"):

                        if (robotData.getCarosel().getShotSuccessInstant() || robotData.getCarosel().isFailsafeSubmode() || !robotData.getCarosel().detectedArtifact()){
                            robotData.getCarosel().endFailsafeSubmode();
                            robotData.getCarosel().incrementRapidFireCurrentShotCount();
                            robotData.getCarosel().activateCycleInProg();
                            robotData.getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
                            break;
                        }

                        robotData.getCarosel().subModeTransferStartTimer();
                        robotData.getCarosel().checkShotSuccess();
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
                                robotData.getCarosel().resetShotSuccess();
                                robotData.getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
                                robotData.getCarosel().activateTransferInProg();
                            }

                        }

                        break;

                    case ("transfer"):

                        if (robotData.getCarosel().getShotSuccessInstant() || robotData.getCarosel().isFailsafeSubmode() || !robotData.getCarosel().detectedArtifact()){
                            robotData.getCarosel().endFailsafeSubmode();
                            robotData.getCarosel().incrementSortedFireCurrentShotCount();
                            robotData.getCarosel().activatePatternInProg();
                            robotData.getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
                            break;
                        }

                        robotData.getCarosel().subModeTransferStartTimer();
                        robotData.getCarosel().checkShotSuccess();
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
        //robotData.getTurret().telemetryTurret(limelight.getDisp());
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

        telemetry.update();
    }

    //Runs at End
    @Override
    public void stop(){
        limelight.killLimeLight();
    }
}