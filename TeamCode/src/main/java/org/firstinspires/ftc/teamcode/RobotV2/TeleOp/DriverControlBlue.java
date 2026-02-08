package org.firstinspires.ftc.teamcode.RobotV2.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotV2.ClassData.LimeLightVision;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RoadRunnerDataV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotConstantsV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotDataV2;

import java.util.List;

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

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        RobotConstantsV2.CAROSEL_TOLERANCE = RobotConstantsV2.CAROSEL_TOLERANCE_TELE;
        RobotConstantsV2.FAILSAFE_SUBMODE_TIMER = RobotConstantsV2.FAILSAFE_SUBMODE_TIMER_TELE; //0
        RobotConstantsV2.COOLDOWN_SHOT = RobotConstantsV2.COOLDOWN_SHOT_TELE; //100
        RobotConstantsV2.COOLDOWN_PRE_SHOT = RobotConstantsV2.COOLDOWN_PRE_SHOT_TELE; //0

        robotData = new RobotDataV2(hardwareMap, telemetry);
        rrData = new RoadRunnerDataV2(robotData);
        rrData.createDashboard();
        limelight = new LimeLightVision(hardwareMap,telemetry,"blue");

        /** Road Runner Init */
        rrData.setBeginPoseFromAuto();
        rrData.createDrive();

        RobotConstantsV2.CAROSEL_TOUCHPAD = 0;

        limelight.initLimeLight();
        robotData.getDriveTrain().setGamepad(gamepad1);
        robotData.getCarosel().indicatorsInInit();
        rrData.updateRobotData(robotData);
    }

    @Override
    public void init_loop(){

        if (!RoadRunnerDataV2.isAutoPosStored) rrData.updateGlobalRobotPosition(limelight);

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
        robotData.getCarosel().cycleOrigin();

        robotData.getTurret().switchTurretMode();
        if (!RoadRunnerDataV2.isAutoPosStored) rrData.updateGlobalRobotPosition(limelight);
    }

    //Loops after Start
    @Override
    public void loop(){

        //Loop Time
        if (gamepad2.leftBumperWasPressed()){
            testLoopTime.reset();
            loopCheck = true;
        }

        /** Important Constant Updates */
        robotData.getDriveTrain().setGamepad(gamepad1);
        robotData.getCarosel().updateCaroselEncoder();
        robotData.updateTelemetry(telemetry);

        /** Main Mode */
        switch (robotData.getDriveTrain().getCurrentMode()){
                //Auto Aim
                case ("auto"):

                    //Direction
                    robotData.getTurret().deactivateHumanIntakeMode();

                    //Switch Mode

                    if (gamepad1.dpadDownWasPressed()){
                        robotData.getTurret().switchAutoSetPos();
                    }

                    //Passive Auto Aim
                    if (robotData.getTurret().isToggleTurretAim()){
                        if (!robotData.getTurret().isAutoSetPosActive()){
                            robotData.getTurret().aimBall(rrData.getDispTotality(limelight));
                        }

                        else{
                            if (robotData.getTurret().isFarToggled()){
                                robotData.getTurret().aimBall(RobotConstantsV2.FAR_BALL_DISTANCE);
                            }
                            else{
                                robotData.getTurret().aimBall(RobotConstantsV2.CLOSE_BALL_DISTANCE);
                            }
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
//                        if (gamepad1.touchpad){
//                            RobotConstantsV2.CAROSEL_TOUCHPAD = (int)(gamepad1.touchpad_finger_1_x * RobotConstantsV2.caroselMultiplier);
//                            robotData.getCarosel().cycleCarosel(robotData.getCarosel().getCurrentCycle());
//                        }

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

        //Far Shot Switches
        if (gamepad1.optionsWasPressed()){
            robotData.getTurret().switchTurretFar();
        }

        /** Switch Turret Mode */
        if (gamepad1.circleWasPressed()){
            robotData.getTurret().switchTurretMode();
        }

        /** Mode Switching */
        if (gamepad1.shareWasPressed()){
            robotData.getDriveTrain().switchMode();
        }

        else if (gamepad1.dpadUpWasPressed() && robotData.getDriveTrain().getCurrentSubMode().equals("none")){
            robotData.getDriveTrain().switchHumanIntake();
        }

        //Rapid Fire
        if (gamepad1.dpadLeftWasPressed()){
            robotData.getDriveTrain().requestRapidFire();
        }

        /** Road Runner */

        //Toggle Auto Aim
        if (gamepad1.left_trigger > RobotConstantsV2.TRIGGER_TOLERENCE){
            robotData.getDriveTrain().forceRobotCentricOff();
        }

        else{
            robotData.getDriveTrain().forceRobotCentricOn();
        }

        //Auto Align Request
//        else if (gamepad1.left_trigger > RobotConstantsV2.TRIGGER_TOLERENCE){
//            rrData.requestAlign(rrData.getYawTotality(limelight));
//            rrData.runCurrentTeleOpAction();
//        }

        //Auto Park Request
         if (gamepad1.right_trigger > RobotConstantsV2.TRIGGER_TOLERENCE){
            rrData.requestPark(limelight.getAlliance());
            rrData.runCurrentTeleOpAction();
        }

        //Reset Action Progress
        else{
            rrData.setTeleOpActionActive(false);
            robotData.getDriveTrain().omniDrive(rrData,limelight);
        }

        /** Sub Modes */

        //Intake
        if (gamepad1.crossWasPressed()){
            robotData.getCarosel().switchIntake();
        }

        //Reverse Intake
        else if (gamepad1.triangleWasPressed()){
            robotData.getCarosel().switchReverseIntake();
        }

        /** Sub Modes */

        //Transfer
        if (gamepad1.rightBumperWasPressed()){
            robotData.getCarosel().transferStartTimer();
        }

        //-----------------------------------------------------

        /** Sub Modes Requested */

        robotData.getDriveTrain().executeSubMode();
        robotData.getCarosel().transferReceiveTimer();

        if (gamepad1.leftBumperWasPressed()){
            LimeLightVision.isFoundMotif = false;
            robotData.getDriveTrain().endSubMode();
            robotData.getCarosel().wipeInventory();
        }

        //-----------------------------------------------------

        /** Limelight & Localization */

        //Vision
        if (limelight.canSeeSomeAT()){
            telemetry.addLine("Eyes Open");
        }
        else{
            telemetry.addLine("Eyes Closed");
        }

        //Failsafe
        if (!(LimeLightVision.isFoundMotif)){
            LimeLightVision.failsafeMotif();
            limelight.updateMotifCode();
        }

        limelight.updateOrientationIMU(rrData.getYaw());
        rrData.updateGlobalRobotPosition(limelight);

        /** Data */
        robotData.getDriveTrain().telemetryDriveTrain();
        robotData.getCarosel().telemetryCarosel();
        robotData.getCarosel().updatePattern(LimeLightVision.motifCode);
        robotData.getDriveTrain().updateModeColor();
        robotData.getDriveTrain().checkEndgame();

        /** Loop Time Check */
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
        robotData.getCarosel().killColorSensors();
    }
}