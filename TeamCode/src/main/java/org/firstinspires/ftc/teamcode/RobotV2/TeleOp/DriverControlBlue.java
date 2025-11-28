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
        if (robotData.getDriveTrain().getCurrentSubMode().equals("none")){
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
                            robotData.getTurret().aimBall(1.7901);
                            telemetry.addLine("Eyes Closed");
                        }

                    }
                    else{
                        robotData.getTurret().killShooter();
                    }

                    if (gamepad1.dpadRightWasPressed()){
                        robotData.getDriveTrain().requestSortedFire(robotData.getCarosel());
                    }

                    //Auto Cycling
                    robotData.getCarosel().autoIntakeCycle();

                    //Indicators
                    robotData.getCarosel().updateIndicators(robotData.getDriveTrain().getCurrentMode(),limelight.getDisp(),robotData.getTurret(),limelight);

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
                            robotData.getCarosel().updateIndicators(robotData.getDriveTrain().getCurrentMode(),RobotConstantsV2.FAR_BALL_DISTANCE,robotData.getTurret(),limelight);
                        }
                        else{
                            robotData.getTurret().aimBall(RobotConstantsV2.CLOSE_BALL_DISTANCE);
                            robotData.getCarosel().updateIndicators(robotData.getDriveTrain().getCurrentMode(),RobotConstantsV2.CLOSE_BALL_DISTANCE,robotData.getTurret(),limelight);
                        }
                    }
                    else{
                        robotData.getTurret().killShooter();
                    }

                    //Cycling

                    if (gamepad1.touchpad_finger_1){
                        RobotConstantsV2.updateCaroselPos((int)(gamepad1.touchpad_finger_1_x * RobotConstantsV2.caroselMultiplier));
                        telemetry.addData("Increment:", (int)(gamepad1.touchpad_finger_1_x * RobotConstantsV2.caroselMultiplier));
                        telemetry.addData("Position 1: ", RobotConstantsV2.caroselPos[0]);
                        robotData.getCarosel().cycleCarosel(robotData.getCarosel().getCurrentCycle());
                    }

                    if (gamepad1.squareWasPressed()){
                        robotData.getCarosel().cycleCaroselManual();
                    }

                    break;

                case("humanIntake"):
                    if (robotData.getTurret().isToggleTurretAim()){
                        robotData.getTurret().activateHumanIntakeMode(robotData.getCarosel());
                    }
                    else{
                        robotData.getTurret().killShooter();
                        robotData.getTurret().deactivateHumanIntakeMode();
                    }

                    robotData.getCarosel().updateIndicators(robotData.getDriveTrain().getCurrentMode(),0,robotData.getTurret(), limelight);
                    break;

                default:
                    telemetry.addLine("Richie is Bald");
                    break;
            }
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
        else if (gamepad1.dpadUpWasPressed()){
            robotData.getDriveTrain().switchHumanIntake();
        }

        /** Auto Park */

        if (gamepad1.left_trigger > 0.75 && gamepad1.right_trigger > 0.75){
            rrData.addTeleOpAction(rrData.getTestSeqAction()); //Replace with Auto Park
            rrData.runTeleOpActions();
        }

        /** Sub Modes */

        if (gamepad1.dpadLeftWasPressed()){
            robotData.getDriveTrain().requestRapidFire(robotData.getCarosel());
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

                        robotData.getCarosel().cycleRapidFire();

                        if (robotData.getCarosel().isCaroselInPlace()){
                            robotData.getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
                            robotData.getCarosel().activateTransferInProg();
                        }

                        break;

                    case ("transfer"):

                        robotData.getCarosel().subModeTransferStartTimer();
                        robotData.getCarosel().checkShotSuccess();

                        if (robotData.getCarosel().getShotSuccessInstant() && !robotData.getCarosel().isTransferCooldownActive()){ //TODO might have to do time based here
                            robotData.getCarosel().resetShotSuccess();
                            robotData.getCarosel().incrementRapidFireCurrentShotCount();
                            robotData.getCarosel().activateCycleInProg();
                            robotData.getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
                        }

                        break;

                    default:
                        break;

                }

                break;

            case ("sortedFire"):
                switch (robotData.getCarosel().getCurrentSubModeQueue()){
                    case ("cycle"):

                        if (robotData.getCarosel().getSortedFireCurrentShotCount() >= RobotConstantsV2.RAPID_FIRE_MAX_SHOTS) {
                            robotData.getDriveTrain().endSubMode();
                            break;
                        }

                        robotData.getCarosel().cycleSortedFire(robotData.getCarosel().getSortedFireCurrentShotCount());

                        if (robotData.getCarosel().isCaroselInPlace()){
                            robotData.getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
                            robotData.getCarosel().activateTransferInProg();
                        }

                        break;

                    case ("transfer"):

                        robotData.getCarosel().subModeTransferStartTimer();
                        robotData.getCarosel().checkShotSuccess();

                        if (robotData.getCarosel().getShotSuccessInstant() && !robotData.getCarosel().isTransferCooldownActive()){
                            robotData.getCarosel().resetShotSuccess();
                            robotData.getCarosel().incrementSortedFireCurrentShotCount();
                            robotData.getCarosel().activatePatternInProg();
                            robotData.getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
                        }

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

        //-----------------------------------------------------

        /** Last Calls */
        if (Arrays.equals(LimeLightVision.motifCode,new String[]{"Empty","Empty","Empty"})){
            limelight.updateMotifCode();        //Very smart ngl
        }

        robotData.getCarosel().updatePattern(LimeLightVision.motifCode);
        robotData.getDriveTrain().updateModeColor(robotData.getTurret());
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