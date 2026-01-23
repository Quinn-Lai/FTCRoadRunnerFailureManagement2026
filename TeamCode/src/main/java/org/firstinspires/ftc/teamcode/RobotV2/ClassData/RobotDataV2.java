package org.firstinspires.ftc.teamcode.RobotV2.ClassData;

import android.graphics.Color;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Arrays;

public class RobotDataV2 {

    /** Components */
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private static ElapsedTime runtime;

    /** Auto Status */
    private boolean pendingSide;
    private boolean pendingPosition;
    private boolean isStartLeft;
    private boolean isStartFar;
    private boolean awaitingTrajectoryGeneration;

    /** Main Classes */
    private DriveTrain driveTrain;
    private Turret turret;
    private Carosel carosel;

    /** Open CV */
    private boolean openCVEnabled;

    //----------------------------------------

    /** Constructor */
    public RobotDataV2(HardwareMap hardwareMap, Telemetry telemetry){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        pendingSide = true;
        pendingPosition = true;
        isStartLeft = true;
        isStartFar = true;

        turret = new Turret();
        carosel = new Carosel();
        driveTrain = new DriveTrain();

        runtime = null;
        awaitingTrajectoryGeneration = true;
    }

    //----------------------------------------

    /** Main Classes */

    public Carosel getCarosel(){
        return carosel;
    }
    public DriveTrain getDriveTrain(){
        return driveTrain;
    }
    public Turret getTurret() {
        return turret;
    }

    /** Components */

    public static void createRuntime(){
        runtime = new ElapsedTime();
    }
    public static double getRuntime(){
        if (runtime != null){
            return runtime.seconds();
        }

        return (new ElapsedTime()).seconds();
    }
    public boolean getOpenCVEnabled() {
        return openCVEnabled;
    }
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }
    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    public void updateTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    public Telemetry getTelemetry(){
        return telemetry;
    }

    /** Auto Status */
    public boolean isPendingSide(){
        return pendingSide;
    }
    public boolean isPendingPosition(){
        return pendingPosition;
    }
    public void selectedSide(){
        pendingSide = false;
    }
    public void selectedPosition(){
        pendingPosition = false;
    }
    public void setStartLeft(boolean pendingPosition){
        isStartLeft = pendingPosition;
    }
    public void setStartFar(boolean far){
        isStartFar = far;
    }
    public boolean isStartFar(){
        return isStartFar;
    }
    public boolean isStartedLeft(){
        return isStartLeft;
    }
    public String getStartingPosition(){
        if (isStartLeft){
            return "Left";
        }

        return "Right";
    }
    public String getStartingSide(){
        if (isStartFar){
            return "Far Side";
        }

        return "Close Side";
    }
    public boolean isAwaitingTrajectoryGeneration(){
        return awaitingTrajectoryGeneration;
    }
    public void generatedTrajectory(){
        awaitingTrajectoryGeneration = false;
    }

    //----------------------------------------

    public class DriveTrain{

        /** Drive Train */
        private DcMotor LFmotor;
        private DcMotor RFmotor;
        private DcMotor LBmotor;
        private DcMotor RBmotor;

        /** Sensors */
        private IMU imu;

        /** Driver Status */
        private boolean robotCentric;
        private Gamepad driver;

        /** Modes */
        private String currentMode;
        private String currentSubMode;

        //----------------------------------------

        /** Constructor */

        public DriveTrain(){ //TODO removed hardware map, hopefully no issue later

            /** Init Hardware */

            LFmotor  = hardwareMap.get(DcMotor.class, "LFmotor");
            RFmotor = hardwareMap.get(DcMotor.class, "RFmotor");
            LBmotor = hardwareMap.get(DcMotor.class, "LBmotor");
            RBmotor = hardwareMap.get(DcMotor.class, "RBmotor");

            LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            RFmotor.setDirection(DcMotor.Direction.FORWARD);
            RBmotor.setDirection(DcMotor.Direction.FORWARD);
            LFmotor.setDirection(DcMotor.Direction.REVERSE);
            LBmotor.setDirection(DcMotor.Direction.REVERSE);

            /** Init Starting Statuses */

            robotCentric = true;
            openCVEnabled = false;

            currentMode = RobotConstantsV2.mainModes[0];
            currentSubMode = RobotConstantsV2.subModes[2];

            /** Field Centric */
            if (!robotCentric){
                imu = hardwareMap.get(IMU.class, "imu");
                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));
                imu.initialize(parameters);
            }
        }

        //----------------------------------------

        /** Main Modes */
        public void switchMode(){
            carosel.resetBooleans();

            if (currentMode.equals(RobotConstantsV2.mainModes[0])){
                currentMode = RobotConstantsV2.mainModes[1];
                carosel.wipeInventory();
            }
            else currentMode = RobotConstantsV2.mainModes[0]; //Default to Auto Mode (Even if humanIntake)
        }
        public void switchHumanIntake(){
            carosel.resetBooleans();
            if (currentMode.equals(RobotConstantsV2.mainModes[2])){
                currentMode = RobotConstantsV2.mainModes[0];
            }
            else{
                currentMode = RobotConstantsV2.mainModes[2];
            }
        }
        public String getCurrentMode(){
            return currentMode;
        }

        /** Sub Modes */
        public String getCurrentSubMode(){
            return currentSubMode;
        }
        public void requestRapidFire(){
            if (turret.isToggleTurretAim() && !driveTrain.getCurrentMode().equals(RobotConstantsV2.mainModes[2])){
                //carosel.wipeInventory(); //Prevent detection issues later (also acts as a reset)
                turret.deactivateHumanIntakeMode();
                carosel.activateCycleInProg();
                carosel.setSubModeQueue(RobotConstantsV2.subModeStages[0]);
                currentSubMode = RobotConstantsV2.subModes[0];
            }
        }
        public void requestSortedFire(){
            if (turret.isToggleTurretAim() && !driveTrain.getCurrentMode().equals(RobotConstantsV2.mainModes[2])){
                turret.deactivateHumanIntakeMode();
                carosel.activatePatternInProg();
                carosel.setSubModeQueue(RobotConstantsV2.subModeStages[0]);
                currentSubMode = RobotConstantsV2.subModes[1];
                carosel.setMaxShotsPattern();
            }
        }
        public void endSubMode(){
            currentSubMode = RobotConstantsV2.subModes[2];
        }
        public void executeSubMode(){
            switch(getCurrentSubMode()){
                //Rapid Fire
                case ("rapidFire"):

                    if (getCarosel().getRapidFireCurrentShotCount() >= RobotConstantsV2.RAPID_FIRE_MAX_SHOTS && !getCarosel().isTransferCooldownActive()) {
                        getCarosel().cycleOrigin();
                        endSubMode();
                        break;
                    }

                    switch (getCarosel().getCurrentSubModeQueue()){
                        case ("cycle"):

                            if (!getCarosel().isTransferCooldownActive()){
                                getCarosel().cycleRapidFire();

                                if (getCarosel().isCaroselInPlace()){
                                    getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
                                    getCarosel().activateTransferInProg();
                                    getCarosel().startPreTransferCooldown();
                                }
                            }

                            break;

                        case ("transfer"):

                            getCarosel().subModeTransferStartTimer();
                            //|| getCarosel().getShotSuccessInstant()!getCarosel().detectedArtifact()  ||
                            if (getCarosel().isFailsafeSubmode()){
                                getCarosel().endFailsafeSubmode();
                                getCarosel().incrementRapidFireCurrentShotCount();
                                getCarosel().activateCycleInProg();
                                getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
                                break;
                            }

                            break;

                        default:
                            break;

                    }

                    break;

                case ("sortedFire"):

                    //getCarosel().updateInventory();

                    if (getCarosel().getSortedFireCurrentShotCount() >= getCarosel().getMaxShots() && !getCarosel().isTransferCooldownActive()) {
                        getDriveTrain().endSubMode();
                        getCarosel().cycleOrigin();
                        break;
                    }

                    switch (getCarosel().getCurrentSubModeQueue()) {
                        case ("cycle"):

                            if (!getCarosel().isTransferCooldownActive()) {

                                getCarosel().cycleSortedFire(getCarosel().getSortedFireCurrentShotCount());

                                if (getCarosel().isCaroselInPlace()) {
                                    //getCarosel().resetShotSuccess();
                                    getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
                                    getCarosel().activateTransferInProg();
                                    getCarosel().startPreTransferCooldown();
                                }
                            }

                            break;

                        case ("transfer"):

                            getCarosel().subModeTransferStartTimer();

                            if (getCarosel().isFailsafeSubmode()){
                                getCarosel().endFailsafeSubmode();
                                getCarosel().incrementSortedFireCurrentShotCount();
                                getCarosel().activatePatternInProg();
                                getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
                                break;
                            }

                            break;

                        default:
                            break;
                    }

                    break;

                default:
                    getCarosel().resetRapidFireCurrentShotCount();
                    getCarosel().resetSortedFireCurrentShotCount();
                    break;

            }
        }
        public boolean executePatternFireAuto(double distance){
            getCarosel().updateCaroselEncoder();
            getTurret().aimBall(distance);

            if (getCarosel().getSortedFireCurrentShotCount() >= getCarosel().getMaxShots() && !getCarosel().isTransferCooldownActive()) {
                getCarosel().transferReceiveTimer();
                getCarosel().resetSortedFireCurrentShotCount();
                getDriveTrain().endSubMode();
                getCarosel().cycleOrigin();
                return false;
            }

            switch (getCarosel().getCurrentSubModeQueue()) {
                case ("cycle"):

                    if (!getCarosel().isTransferCooldownActive()) {

                        getCarosel().cycleSortedFire(getCarosel().getSortedFireCurrentShotCount());

                        if (getCarosel().isCaroselInPlace()) {
                            //getCarosel().resetShotSuccess();
                            getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
                            getCarosel().activateTransferInProg();
                            getCarosel().startPreTransferCooldown();
                        }
                    }

                    break;

                case ("transfer"):
                    getCarosel().subModeTransferStartTimer();

                    if (getCarosel().isFailsafeSubmode()){
                        getCarosel().endFailsafeSubmode();
                        getCarosel().incrementSortedFireCurrentShotCount();
                        getCarosel().activatePatternInProg();
                        getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
                        break;
                    }

                    break;

                default:
                    break;
            }

            return true;
        }
        public boolean executePatternFireAutoFar(double distance){
            getCarosel().updateCaroselEncoder();
            getTurret().aimBall(distance);

            if (getCarosel().getSortedFireCurrentShotCount() >= getCarosel().getMaxShots() && !getCarosel().isTransferCooldownActive()) {
                getCarosel().transferReceiveTimer();
                getCarosel().resetSortedFireCurrentShotCount();
                getDriveTrain().endSubMode();
                getCarosel().cycleOrigin();
                return false;
            }

            switch (getCarosel().getCurrentSubModeQueue()) {
                case ("cycle"):

                    if (!getCarosel().isTransferCooldownActive()) {

                        getCarosel().cycleSortedFire(getCarosel().getSortedFireCurrentShotCount());

                        if (getCarosel().isCaroselInPlace()) {
                            //getCarosel().resetShotSuccess();
                            getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
                            getCarosel().activateTransferInProg();
                            getCarosel().startPreTransferCooldown();
                        }
                    }

                    break;

                case ("transfer"):

                    boolean status = getTurret().isUpToSpeed(RobotConstantsV2.FAR_BALL_DISTANCE);
                    if (status) getCarosel().subModeTransferStartTimer();

                    if (getCarosel().isFailsafeSubmode()){
                        getCarosel().endFailsafeSubmode();
                        getCarosel().incrementSortedFireCurrentShotCount();
                        getCarosel().activatePatternInProg();
                        getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
                        break;
                    }

                    break;

                default:
                    break;
            }

            return true;
        }
        public boolean executeRapidFireAuto(double distance){
            getCarosel().updateCaroselEncoder();
            getTurret().aimBall(distance);

            if (getCarosel().getRapidFireCurrentShotCount() >= RobotConstantsV2.RAPID_FIRE_MAX_SHOTS && !getCarosel().isTransferCooldownActive()) {
                getCarosel().transferReceiveTimer();
                getCarosel().resetRapidFireCurrentShotCount();
                getDriveTrain().endSubMode();
                getCarosel().cycleOrigin();
                return false;
            }

            switch (getCarosel().getCurrentSubModeQueue()){
                case ("cycle"):

                    if (!getCarosel().isTransferCooldownActive()){
                        getCarosel().cycleRapidFire();

                        if (getCarosel().isCaroselInPlace()){
                            //getCarosel().resetShotSuccess();
                            getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
                            getCarosel().activateTransferInProg();
                            getCarosel().startPreTransferCooldown();
                        }
                    }

                    break;

                case ("transfer"):

                    getCarosel().subModeTransferStartTimer();

                    if (getCarosel().isFailsafeSubmode()){
                        getCarosel().endFailsafeSubmode();
                        getCarosel().incrementRapidFireCurrentShotCount();
                        getCarosel().activateCycleInProg();
                        getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
                        break;
                    }

                    break;

                default:
                    break;

            }

            return true;
        }


        /** Movement */
        public void powerMotorSafe(DcMotor motor, double targetPower){
            final double SLEW = 0.2;
            double currentPower = motor.getPower();

            double desiredChange = targetPower - currentPower;
            double limitedChange = Math.max(-SLEW, Math.min(desiredChange, SLEW));

            motor.setPower(currentPower += limitedChange);
        }
        public void omniDrive(){

            double x = driver.left_stick_x;
            double y = -driver.left_stick_y;
            double z = driver.right_stick_x;

            //Robot POV
            if (robotCentric){
                x *= 1.1;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(z), 1);
                double frontLeftPower = (y + x + z) / denominator;
                double backLeftPower = (y - x + z) / denominator;
                double frontRightPower = (y - x - z) / denominator;
                double backRightPower = (y + x - z) / denominator;

                LFmotor.setPower(frontLeftPower);
                LBmotor.setPower(backLeftPower);
                RFmotor.setPower(frontRightPower);
                RBmotor.setPower(backRightPower);
            }

            //Field POV
            else{
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX *= 1.1;

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(z), 1);
                double frontLeftPower = (rotY + rotX + z) / denominator;
                double backLeftPower = (rotY - rotX + z) / denominator;
                double frontRightPower = (rotY - rotX - z) / denominator;
                double backRightPower = (rotY + rotX - z) / denominator;

                LFmotor.setPower(frontLeftPower);
                LBmotor.setPower(backLeftPower);
                RFmotor.setPower(frontRightPower);
                RBmotor.setPower(backRightPower);
            }
        }
        public void omniDrive(Gamepad gamepad) {

            double x = gamepad.left_stick_x;
            double y = -gamepad.left_stick_y;
            double z = gamepad.right_stick_x;

            //Robot POV
            if (robotCentric) {
                x *= 1.1;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(z), 1);
                double frontLeftPower = (y + x + z) / denominator;
                double backLeftPower = (y - x + z) / denominator;
                double frontRightPower = (y - x - z) / denominator;
                double backRightPower = (y + x - z) / denominator;

                LFmotor.setPower(frontLeftPower);
                LBmotor.setPower(backLeftPower);
                RFmotor.setPower(frontRightPower);
                RBmotor.setPower(backRightPower);
            }
            //Field POV
            else{
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the movement direction counter to the bot's rotation
                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX *= 1.1;

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(z), 1);
                double frontLeftPower = (rotY + rotX + z) / denominator;
                double backLeftPower = (rotY - rotX + z) / denominator;
                double frontRightPower = (rotY - rotX - z) / denominator;
                double backRightPower = (rotY + rotX - z) / denominator;

                LFmotor.setPower(frontLeftPower);
                LBmotor.setPower(backLeftPower);
                RFmotor.setPower(frontRightPower);
                RBmotor.setPower(backRightPower);
            }
        }

        /** Gamepad */
        public void setGamepad(Gamepad driver) {
            this.driver = driver;
        }

        /** Final Updates */
        public void updateModeColor(){
            if (currentMode.equals(RobotConstantsV2.mainModes[0])){
                driver.setLedColor( 0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
            }
            else if (currentMode.equals(RobotConstantsV2.mainModes[1])){
                //Differentiate these two
                if (turret.isFarToggled()){
                    driver.setLedColor( 0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
                }
                else{
                    driver.setLedColor( 0, 1, 1, Gamepad.LED_DURATION_CONTINUOUS);
                }
            }
            else{
                driver.setLedColor( 1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            }
        }
        public void checkEndgame(){
            double currentRuntime = getRuntime();

            if (currentRuntime >= RobotConstantsV2.ENDGAME_MARKER && currentRuntime <= RobotConstantsV2.ENDGAME_MARKER + 2){
                driver.rumble(2);
                telemetry.addLine("END GAME!");
            }

            else if (currentRuntime >= RobotConstantsV2.FINAL_ENDGAME_MARKER && currentRuntime <= RobotConstantsV2.FINAL_ENDGAME_MARKER + 2){
                driver.rumble(2);
                double time = 120 - currentRuntime;
                telemetry.addLine("END GAME!");
                telemetry.addLine(Math.floor(time) + " Seconds Left!");
            }
        }
        public void telemetryDriveTrain(){

            telemetry.addLine("------------------------------------\n");

            telemetry.addLine("Drive Train: ");

            telemetry.addData("Time: ", Math.floor(getRuntime()) + " Seconds");
            telemetry.addData("Current Mode: ", currentMode);
            telemetry.addData("Current Sub Mode: ", currentSubMode);

        }
    }

    //----------------------------------------

    public class PIDControl{

        /** PID For Shooter*/
        private double kP = 0.007;
        private double kI = 0.0000015;
        private double kD = 0.000003;
        private double kF = 0.00045;
        private ElapsedTime PIDTimer = new ElapsedTime();
        private double integralSum = 0;
        private double lastError = 0;

        /** PID for Carosel */
//        private double kPC = 0.0019;
//        private double kIC = 0.000021; //0.000005
//        private double kDC = 0.013; //0.13
        private ElapsedTime PIDTimerC = new ElapsedTime();
        private double integralSumC = 0;
        private double lastErrorC = 0;

        //----------------------------------------

        /** Get Constants for Shooter */
        public String getPIDCos(){
            return String.format("kP: %f, kI: %f, kD: %f, kF: %f", RobotConstantsV2.kPC, RobotConstantsV2.kIC, RobotConstantsV2.kDC, kF);
        }

        /** PID */
        public double PIDShooter(double current, double desired) {

            double currentTime = PIDTimer.seconds();

            double output = 0;

            double error = desired - current;

            integralSum += error * currentTime;

            double derivate = (error - lastError) / currentTime;

            lastError = error;

            PIDTimer.reset();

            output = (error * kP) + (derivate * kD) + (integralSum * kI) + (desired * kF);

            return output;
        }

        public double PIDCarosel(double current, double desired) {

            double currentTimeC = PIDTimer.seconds();

            double outputC = 0;

            double errorC = desired - current;

            integralSumC += errorC * currentTimeC;

            double derivateC = (errorC - lastErrorC) / currentTimeC;

            lastError = errorC;

            PIDTimerC.reset();

            double positionalCalc = errorC * RobotConstantsV2.kPC;
            double integralCalc = integralSumC * RobotConstantsV2.kIC;
            double derivativeCalc = derivateC * RobotConstantsV2.kDC;

            if (positionalCalc > 0.3) positionalCalc = 0.3;
            else if (positionalCalc < -0.3) positionalCalc = -0.3;

            if (integralCalc > 0.1) integralCalc = 0.1;
            else if (integralCalc < -0.1) integralCalc = -0.1;

            if (derivativeCalc > 0.15) derivativeCalc = 0.15;
            else if (derivativeCalc < -0.15) derivativeCalc = -0.15;

            outputC = (positionalCalc) + (derivativeCalc) + (integralCalc);

//            telemetry.addLine(getPIDCos());
//            telemetry.addData("Output P: ", positionalCalc);
//            telemetry.addData("Output I: ", integralCalc);
//            telemetry.addData("Output D: ", derivativeCalc);
//
//            telemetry.addData("Output: ", outputC);
            return outputC;
        }
    }

    //----------------------------------------

    public class Turret extends PIDControl{

        /** Parts */
        public DcMotorEx shooterMotor;
        private Servo shooterServo;

        /** Physics Parameters */
        private final double a = -9.8;
        public double heightOfLauncher = 0.3556; //TODO Update this
        private final double height = RobotConstantsV2.HEIGHT_TO_AIM - heightOfLauncher;
        private final double vY = Math.sqrt(2 * -a * height);
        private double vX = 0;
        private double vX0 = 0; //initial vX of the robot at an instant in time

        /** Fly Wheel */
        private final double shooterWheelRadius = 0.096/2; //Meters
        private final double w = 28.0; //Encoder Resolution for 6K RPM

        /** Toggle Turret */
        private boolean toggleTurretAim;
        private boolean isFar;

        //----------------------------------------

        /** Constructor */
        private Turret(){

            /** Hardware Init */
            shooterMotor = (DcMotorEx)hardwareMap.get(DcMotor.class,"shooterMotor");
            shooterServo = hardwareMap.get(Servo.class,"shooterServo");

            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotor.setDirection(DcMotor.Direction.REVERSE);

            /** Mode Init */
            toggleTurretAim = false;
            isFar = false;
        }

        //----------------------------------------

        /** Physics Calculations */
        private double getYEquation(double time){
            return vY * time + 0.5 * a * Math.pow(time, 2);
        }
        private double getXEquation(double time){
            return vX * time;
        }
        public double getHeightOfLauncher(){
            return heightOfLauncher;
        }
        private double getTimeExtrema(){
            //Using the formula -b/2a, SAME THING as derivative of position equation and the vf = vi + at equation
            return -vY/a; //2's cancel
        }
        private double getVy(){
            return vY ;
        }
        private double getInitalVelocityX(){
            return vX0;
        }
        private double getVx(double disp){
            return ((disp / getTimeExtrema()) + vX0);
        }
        private double getVelocityTotal(double disp){
            return Math.sqrt(Math.pow(getVy(),2) + Math.pow(getVx(disp),2));
        }
        private double getAngleTotal(double disp){
            return Math.atan2(getVy(),getVx(disp)) * (180 / Math.PI);
        }
        private double getShooterWheelRadius(){
            return shooterWheelRadius;
        }
        private double getRadiansPerSecond(double disp){ //From Velocity
            return getVelocityTotal(disp)/ getShooterWheelRadius();
        }
        private double getDesiredRPM(double disp){ //From Radians Per SEcond
            return getRadiansPerSecond(disp) * 60 / (2 * Math.PI);
        }
        private double getW(){
            return w;
        }
        public double getTPS(double disp){ //From Radians per Second

            double TPSfound = ((getRadiansPerSecond(disp) * getW()) / (2 * Math.PI)) * RobotConstantsV2.dragMultiplier;

            if (TPSfound > 1800){
                return 1800;
            }

            return TPSfound;
        }

        /** Turret Modes */
        public void switchTurretMode(){
            if (toggleTurretAim) toggleTurretAim = false;
            else toggleTurretAim = true;
        }
        public boolean isToggleTurretAim(){
            return toggleTurretAim;
        }
        public void switchTurretFar(){
            if (isFar) isFar = false;
            else isFar = true;
        }
        public void toggleTurretFar(boolean isFar){
            this.isFar = isFar;
        }
        public boolean isFarToggled(){
            return isFar;
        }

        /** Human Player Intake Mode */
        public void activateHumanIntakeMode(){
            shooterServo.setPosition(RobotConstantsV2.MIN_HOOD_ANGLE_POS);
            shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
            powerShooterMotor(RobotConstantsV2.HUMAN_INTAKE_SPEED);
        }
        public void deactivateHumanIntakeMode(){
            shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        }

        /** Shooter Power */
        public void powerShooterMotor(double TPS){
            double current = shooterMotor.getVelocity();
            //shooterMotor.setVelocity(PIDShooter(current,TPS));
            shooterMotor.setPower(PIDShooter(current,TPS));
            //telemetry.addData("TPS NOW", TPS);
            //telemetry.addData("TPS Current: ", current);

        }
        public void killShooter(){
            //shooterMotor.setVelocity(RobotConstantsV2.KILL_SHOOTER_SPEED);
            shooterMotor.setPower(0);
        }

        /** Hood */
        private double convertDegToServo(double angle){

            //telemetry.addData("Angle: ", angle);
            if (angle > RobotConstantsV2.MAX_HOOD_ANGLE) return RobotConstantsV2.MAX_HOOD_ANGLE_POS;
            else if (angle < RobotConstantsV2.MIN_HOOD_ANGLE) return RobotConstantsV2.MIN_HOOD_ANGLE_POS;

            return (double) Math.round((0.033358 * angle - 1.41127) * 1000) / 1000;
        }
        public void angleRobot(double disp){
            double desiredAngle = getAngleTotal(disp) + RobotConstantsV2.ANGLE_BONUS;
            shooterServo.setPosition(convertDegToServo(desiredAngle));
        }

        /** Final Shot */
        public void aimBall(double disp){
            angleRobot(disp);
            double TPS = getTPS(disp);
            powerShooterMotor(TPS);
        }
        public void aimBall(double disp, double TPS){
                angleRobot(disp);
                powerShooterMotor(TPS);
        }

        /** Final Updates */
        public void telemetryTurret(double disp){
//            telemetry.addLine("Kinematics: \n");
//
//            telemetry.addData("Total Velocity: ", getVelocityTotal(disp));
//            telemetry.addData("vY: ", getVy());
//            telemetry.addData("vX: ",getVx(disp));
//            telemetry.addData("vX0: ", getInitalVelocityX());
//            telemetry.addData("Estimated Drag Multiplier: ", RobotConstantsV2.dragMultiplier);
//            telemetry.addLine("-------------------------- \n");
//
//            telemetry.addLine("TPS: \n");
//
//            telemetry.addData("True TPS: ", shooterMotor.getVelocity());
//            telemetry.addData("Ticks Per Second: ", getTPS(disp));
//            telemetry.addData("TPS Error:", getTPSError(disp));
//
//            telemetry.addLine("-------------------------- \n");
//
//            telemetry.addLine("Outputs: \n");
//
//            telemetry.addData("Time Extrema:", getTimeExtrema());
//            telemetry.addData("Displacement (m): ", disp);
//            telemetry.addData("Angle: " , getAngleTotal(disp));
//            telemetry.addData("True Servo Angle:",convertDegToServo(getAngleTotal(disp)));
            telemetry.addLine("------------------------------------\n");

            telemetry.addLine("Turret: ");

            telemetry.addData("Current Shooter Motor TPS: ", shooterMotor.getVelocity());
            telemetry.addData("Desired TPS: ", getTPS(disp));
            telemetry.addData("TPS Error:", getTPSError(disp));
            telemetry.addData("Displacement (m): ", disp);
            telemetry.addData("Angle: ", getAngleTotal(disp));

        }
        public void telemetryTurretBasic(){
            telemetry.addData("Turret On: ", toggleTurretAim);
            telemetry.addData("Far Shot: ", !isFar);
        }
        public double getTPSError(double disp){
            return shooterMotor.getVelocity() - getTPS(disp);
        }
        public double getTPSErrorManual(double TPS){
            return shooterMotor.getVelocity() - TPS;
        }
        public boolean isUpToSpeed(double distance){
            //Current error less than allowed error
            return Math.abs(getTPSError(distance)) < getTurret().getTPS(distance) * RobotConstantsV2.SHOOTER_SPEED_THRESHOLD;
        }
    }

    //----------------------------------------

    public class Carosel extends PIDControl{

        /** Hardware */
        private DcMotorEx intakeMotor;
        private Servo transferServo;
        private CRServo caroselServo;
        private Servo caroselServoPos; //TODO temp
        private AnalogInput caroselAnalog;


        /** Sensors */
        private Servo indicatorOne;
        private Servo indicatorTwo;
        private NormalizedColorSensor colorSensorFront;
        private NormalizedColorSensor colorSensorBack;
        private DistanceSensor colorSensorFrontDist;
        private DistanceSensor colorSensorBackDist;
        private AnalogInput rangerSensor;
        private DigitalChannel transferBeam;


        /** Storage */
        private String[] inventory;
        private String[] pattern;
        private int currentCycle;


        /** Intake  */
        private boolean intakeMotorOn;
        private boolean autoIntakeCooldownActive;
        private ElapsedTime autoIntakeCoolDown;
        private boolean ejectActive;

        /** Transfer */
        private boolean transferUp;
        private ElapsedTime transferCooldown;
        private boolean transferCooldownActive;
        private ElapsedTime pretransferCooldown;
        private boolean pretransferCooldownActive;

        /** Submode */
        private boolean cycleInProg;
        private boolean patternInProg;
        private String currentSubModeQueue;
        private boolean transferInProg;
        private int rapidFireCurrentShotCount;
        private int sortedFireCurrentShotCount;
        private boolean shotSucceed;
        private boolean failsafeSubmode;
        private ElapsedTime failsafeTimer;
        private int maxShots;

        /** PID */
        private double lastCaroselPosition;
        private double globalCaroselPosition;

        //----------------------------------------

        /** Constructor */
        public Carosel(){

            /** Hardware Init */

            caroselServo = hardwareMap.get(CRServo.class,"caroselServo");
            caroselServoPos = hardwareMap.get(Servo.class,"caroselServoL"); //TODO temp
            caroselAnalog = hardwareMap.get(AnalogInput.class,"analogCarosel");
            transferServo = hardwareMap.get(Servo.class,"transferServo");

            /** Sensors Init */
            indicatorOne = hardwareMap.get(Servo.class,"indicatorOne");
            indicatorTwo = hardwareMap.get(Servo.class,"indicatorTwo");
            colorSensorFront = hardwareMap.get(NormalizedColorSensor.class,"colorSensorFront");
            colorSensorBack  = hardwareMap.get(NormalizedColorSensor.class,"colorSensorBack");

            colorSensorFront.setGain(RobotConstantsV2.SENSOR_GAIN);
            colorSensorBack.setGain(RobotConstantsV2.SENSOR_GAIN);

            colorSensorFrontDist = hardwareMap.get(DistanceSensor.class,"colorSensorFront");
            colorSensorBackDist = hardwareMap.get(DistanceSensor.class,"colorSensorBack");

            rangerSensor = hardwareMap.get(AnalogInput.class,"rangerSensor");
            transferBeam = hardwareMap.get(DigitalChannel.class,"transferBeam");

            intakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class,"intakeMotor");
            intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);


            /** Spindex */

            //TODO I think this becomes the "zero"
            lastCaroselPosition = caroselAnalog.getMaxVoltage() * RobotConstantsV2.encoderRes; //TODO might need to change logic of this
            globalCaroselPosition = 0;

            /** Inventory */
            inventory = new String[]{"Empty","Empty","Empty"};
            pattern = new String[]{"Empty","Empty","Empty"};

            /** Variable Init */

            autoIntakeCooldownActive = false;
            cycleInProg = false;
            patternInProg = false;
            transferInProg = false;
            intakeMotorOn = false;
            transferUp = false;
            transferCooldownActive = false;
            shotSucceed = false;
            failsafeSubmode = false;
            ejectActive = false;
            pretransferCooldownActive = false;

            /** Timers */
            pretransferCooldown = new ElapsedTime();
            failsafeTimer = new ElapsedTime();
            transferCooldown = new ElapsedTime();
            autoIntakeCoolDown = new ElapsedTime();

            /** Sub Modes */
            rapidFireCurrentShotCount = 0;
            sortedFireCurrentShotCount = 0;
        }

        //----------------------------------------

        public void resetBooleans(){
            autoIntakeCooldownActive = false;
            cycleInProg = false;
            patternInProg = false;
            transferInProg = false;
            transferCooldownActive = false;
            shotSucceed = false;
            failsafeSubmode = false;
        }

        /** Autonomous */
        public void setInventoryAuto(){
            inventory = new String[]{"Green","Purple","Purple"};
        }
        public void forceFeedInventory(String color1, String color2, String color3){
            getCarosel().getInventory()[0] = color1;
            getCarosel().getInventory()[1] = color2;
            getCarosel().getInventory()[2] = color3;
        }


        /** Intake */
        public void forceReverseIntakeOn(){
            intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
            intakeMotor.setVelocity(RobotConstantsV2.INTAKE_IN_EJECT);
            intakeMotorOn = true;
        }
        public void forceReverseIntakeOff(){
            intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
            intakeMotor.setVelocity(RobotConstantsV2.INTAKE_IN_EJECT);
            intakeMotorOn = true;
        }
        public void switchIntake(){
            intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
            if (!intakeMotorOn){
                intakeMotor.setVelocity(RobotConstantsV2.INTAKE_ON);
                intakeMotorOn = true;
            }
            else{
                intakeMotor.setVelocity(RobotConstantsV2.INTAKE_OFF);
                intakeMotorOn = false;
            }
        }
        public void switchReverseIntake(){
            intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
            if (!intakeMotorOn){
                intakeMotor.setVelocity(RobotConstantsV2.INTAKE_IN_EJECT);
                intakeMotorOn = true;
            }
            else{
                intakeMotor.setVelocity(RobotConstantsV2.INTAKE_OFF);
                intakeMotorOn = false;
            }
        }
        public void forceIntakeOn(){
            intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
            intakeMotor.setVelocity(RobotConstantsV2.INTAKE_ON);
            intakeMotorOn = true;
        }
        public void forceIntakeOff(){
            intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
            intakeMotor.setPower(RobotConstantsV2.INTAKE_OFF);
            intakeMotorOn = false;
        }
        public void checkForAutoEject(){
            if (!isEmptySpot() && !ejectActive){
                intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
                intakeMotor.setVelocity(RobotConstantsV2.INTAKE_IN_EJECT);
                intakeMotorOn = true;
                ejectActive = true;
                cycleOrigin();
            }

            else if (isEmptySpot()){
                ejectActive = false;
            }
        }
        public boolean isIntakeMotorOn(){
            return intakeMotorOn;
        }


        /** Transfer */
        public void toggleTransfer(){
            if (!transferUp){
                //inventory[currentCycle] = "Empty";
                transferServo.setPosition(RobotConstantsV2.TRANSFER_UP);
                transferUp = true;
            }
            else{
                transferServo.setPosition(RobotConstantsV2.TRANSFER_DOWN);
                transferUp = false;
            }
        }
        public void forceTransferUp(){
            inventory[currentCycle] = "Empty";
            transferServo.setPosition(RobotConstantsV2.TRANSFER_UP);
            transferUp = true;
        }
        public void forceTransferDown(){
            transferServo.setPosition(RobotConstantsV2.TRANSFER_DOWN);
            transferUp = false;
            transferCooldownActive = false;
        }
        public void transferStartTimer(){
            if (turret.isToggleTurretAim() && !driveTrain.getCurrentMode().equals(RobotConstantsV2.mainModes[2]) && isCaroselInPlace()){
                transferCooldownActive = true;
                forceTransferUp();
                transferCooldown.reset();
            }
        } //TODO simplify
        public void transferReceiveTimer(){
            if (transferCooldownActive && transferCooldown.milliseconds() > RobotConstantsV2.COOLDOWN_SHOT){
                forceTransferDown();
                transferCooldownActive = false;
            }
        }
        public void activateTransferInProg(){
            transferInProg = true;
        }
        public void subModeTransferStartTimer(){
            if (transferInProg && isCaroselInPlace() && pretransferCooldown.milliseconds() > RobotConstantsV2.COOLDOWN_PRE_SHOT){ //
                transferInProg = false;
                transferCooldownActive = true;
                forceTransferUp();
                transferCooldown.reset();
                startFailsafeSubmode();
                pretransferCooldownActive = false;
            }
        }
        public boolean isTransferCooldownActive(){
            return transferCooldownActive;
        }
        public void startPreTransferCooldown(){
            pretransferCooldown.reset();
        }


        /** Sub Modes */
        public int getMaxShots(){
            return maxShots;
        }
        public void setMaxShotsPattern(){

            int count = 0;

            for (String i:inventory){

                if (i.equals("Green") || i.equals("Purple")){
                    count++;
                }
            }

            maxShots = count;
        }
        public void resetMaxShotsPattern(){
            maxShots = 0;
        }
        public void endFailsafeSubmode(){
            failsafeSubmode = false;
        }
        public void startFailsafeSubmode(){
            failsafeSubmode = true;
            failsafeTimer.reset();
        }
        public boolean isFailsafeSubmode(){
            //!shotSucceed &&
            if (failsafeSubmode && failsafeTimer.milliseconds() > RobotConstantsV2.FAILSAFE_SUBMODE_TIMER){
                failsafeSubmode = false;
                return true;
            }
            return false;
        }
        public boolean getTransferInProg(){
            return transferInProg;
        }
        public void checkShotSuccess(){
            if (isShotSuccess()){
                shotSucceed = true;
                //inventory[currentCycle] = "Empty";
            }
        }
        public void resetShotSuccess(){
            shotSucceed = false;
        }
        public boolean getShotSuccessInstant(){
            return shotSucceed;
        }
        public void activateCycleInProg(){
            cycleInProg = true;
        }
        public void activatePatternInProg(){
            patternInProg = true;
        }
        public String getCurrentSubModeQueue(){
            return currentSubModeQueue;
        }
        public void updateSubModeQueue(){
            if (currentSubModeQueue.equals(RobotConstantsV2.subModeStages[0])){
                currentSubModeQueue = RobotConstantsV2.subModeStages[1];
            }
            else{
                currentSubModeQueue = RobotConstantsV2.subModeStages[0];
            }
        }
        public void setSubModeQueue(String queue){
            currentSubModeQueue = queue;
        }
        public int getAvailableShots(){
            int shotCounter = 0;

            for (String i:inventory){
                if (i.equals("Green") || i.equals("Purple")){
                    shotCounter ++;
                }
            }

            return shotCounter;
        }
        public int getRapidFireCurrentShotCount(){
            return rapidFireCurrentShotCount;
        }
        public int getSortedFireCurrentShotCount(){
            return sortedFireCurrentShotCount;
        }
        public void resetSortedFireCurrentShotCount(){
            sortedFireCurrentShotCount = 0;
        }
        public void incrementSortedFireCurrentShotCount(){
            sortedFireCurrentShotCount++;
        }
        public void incrementRapidFireCurrentShotCount(){
            rapidFireCurrentShotCount ++;
        }
        public void resetRapidFireCurrentShotCount(){
            rapidFireCurrentShotCount = 0;
        }
        public boolean isReadyShoot(){
            return !inventory[currentCycle].equals("Empty");
        }
        private String getTransferColor(){
            return inventory[currentCycle];
        }
        public boolean isEmptySpot(){
            return Arrays.asList(inventory).contains("Empty");
            //return !(getEmptySpot() == -1);
        }
        public boolean isShotSuccess(){
            return transferBeam.getState();
        }
        public void removeArtifactPostShot(){ //TODO beta test this
            if (isShotSuccess()) inventory[currentCycle] = "Empty";
        }

        /** Pattern & Inventory*/
        public void updatePattern(String[] pattern){
            this.pattern = pattern;
        }
        public boolean hasPattern() {

            String[] tempArrayInventory = Arrays.copyOf(inventory,inventory.length);
            String[] tempArrayPattern = Arrays.copyOf(pattern,pattern.length);

            Arrays.sort(tempArrayInventory);
            Arrays.sort(tempArrayPattern);

            return Arrays.equals(tempArrayInventory,tempArrayPattern);
        }
        public void updateInventory(){
            //Won't override updates
            //TODO remove transfer in prog if its buggy
            //inventory[currentCycle].equals("Green") || inventory[currentCycle].equals("Purple") ||
            if (!isCaroselInPlace()){
                return;
            }

            if (inventory[currentCycle].equals("Empty")){
                inventory[currentCycle] = getColorFront();
            }

            if (inventory[currentCycle].equals("Green") || inventory[currentCycle].equals("Purple")){
                if (getColorBack().equals("Empty")) return;
                inventory[currentCycle] = getColorBack();
            }

            //inventory[currentCycle] = getColorFront();

        }
        public void wipeInventory(){
            inventory = new String[]{"Empty","Empty","Empty"};
        }
        public int getEmptySpot(){
            return Arrays.asList(inventory).indexOf("Empty");
        }
        public int getInvPosition(String color){

            int inventoryCount = 0;

            for (String i: inventory){
                if (i.equals(color)){
                    return inventoryCount;
                }
                else{
                    inventoryCount++;
                }
            }

            return -1;
        }
        public String[] getInventory(){
            return inventory;
        }

        /** Cycling */
        public void cycleRapidFire(){
            //Only Activates Once
            //TODO incrementRapidFireCurrentShotCount(); TRANSFER
            if (cycleInProg){
                if (getCarosel().getRapidFireCurrentShotCount() != 0) cycleCaroselManual();
                cycleInProg = false;
            }
        }
        public void cycleSortedFire(int patternNum){
            if (patternInProg && patternNum <= 2) {
                cyclePattern(patternNum);
                patternInProg = false;
            }
        }
        public void cycleCaroselManual(){
            cycleCarosel(getNextCycle());
        }
        public void autoIntakeCycle(){
            //telemetry.addData("Next Empty Spot: ", getEmptySpot());
            updateCaroselEncoder();
            if (!autoIntakeCooldownActive && !transferCooldownActive && detectedArtifact() && isCaroselInPlace() && isEmptySpot()){
                updateInventory();
                autoIntakeCooldownActive = true;
                autoIntakeCoolDown.reset();
            }
        }
        public boolean isAutoIntakeCooldownActive(){
            return autoIntakeCooldownActive;
        }
        public void receiveAutoCycleStatus(){
            if (autoIntakeCooldownActive && autoIntakeCoolDown.milliseconds() > RobotConstantsV2.AUTO_CYCLE_COOLDOWN){
                autoIntakeCooldownActive = false;

                if (inventory[currentCycle].equals("Empty") && detectedArtifact()){
                    inventory[currentCycle] = "Purple"; //Failsafe
                }


                if (isEmptySpot()) cycleCarosel(getEmptySpot());
            }
        }
        public void cycleOrigin(){
            cycleCarosel(0);
        }
//        public void cycleCarosel(int desiredCycle){
//            if (!transferCooldownActive){
//                int increm = getCycleIncrement(desiredCycle) + RobotConstantsV2.CAROSEL_TOUCHPAD;
//                currentCycle = desiredCycle;
//
//                telemetry.addData("Run to Pos: ", increm);
//
//                runCaroselToPos(increm);
//            }
//        }

        public void cycleCarosel(int desiredCycle){
            if (!transferCooldownActive){
                double increm = RobotConstantsV2.caroselPositions[desiredCycle] + RobotConstantsV2.CAROSEL_TOUCHPAD / 100.0;
                currentCycle = desiredCycle;
                caroselServoPos.setPosition(increm);
            }
        }

        public boolean isCompleteEmpty(){
            return inventory[0].equals("Empty") && inventory[1].equals("Empty") && inventory[2].equals("Empty");
        }

        public void cyclePattern(int number){
            int slot = getInvPosition(pattern[number]);
            //int trashArtifact = getNearestUselessArtifact();

            if (isCompleteEmpty()) return;

            if (slot == -1){
                //if (trashArtifact != -1) cycleCarosel(trashArtifact);
                if (maxShots > sortedFireCurrentShotCount) cycleCarosel(getNearestArtifact());
                return;
            }

            cycleCarosel(slot);
        }


        /** Carosel Servo Logic */
        public double getCaroselDegrees(){
            return caroselAnalog.getVoltage() * RobotConstantsV2.encoderRes;
        }

//        public boolean isBusy(){
//            double targetPos = getCycleIncrement(currentCycle) + RobotConstantsV2.CAROSEL_TOUCHPAD; //TODO test this
//            return Math.abs(targetPos - getCaroselDegrees()) < RobotConstantsV2.CAROSEL_TOLERANCE;
//        }
//
        public boolean isBusy(){ //TODO Temp
            double targetPos = -318.91011 * (RobotConstantsV2.caroselPositions[currentCycle] + RobotConstantsV2.CAROSEL_TOUCHPAD/100.0) + 337.27894;
            return Math.abs(targetPos - globalCaroselPosition) > RobotConstantsV2.CAROSEL_TOLERANCE;
        }


        public boolean isCaroselInPlace(){
            return !isBusy();
        }
        private void runCaroselToPos(double pos){
            caroselServo.setPower(-PIDCarosel(globalCaroselPosition,pos));
        }
        private double getCaroselIncrement(double diff, double current){

            double step = 0;

            double N = 360;

            double diffe = diff - current;

            double modDiff = ((diffe % N) + N) % N;

            if (modDiff > N/ 2){
                step = (modDiff - N);
            }
            else{
                step = (modDiff);
            }

            return step;
        } //this is used for tracking global encoder pos

//        public void updateCaroselEncoder(){
//            double current = getCaroselDegrees();
//            double in = getCaroselIncrement(current,lastCaroselPosition);
//            lastCaroselPosition = current;
//            globalCaroselPosition += in;
//            telemetry.addData("globalCaroselPos: ", globalCaroselPosition);
//        }

        public void updateCaroselEncoder(){ //TODO Temp
            globalCaroselPosition = caroselAnalog.getVoltage() * RobotConstantsV2.encoderRes;
        }

        public void resetCaroselGlobalIncrement(){
            //TODO Find a way to zero it
            RobotConstantsV2.CAROSEL_GLOBAL_INCREMENT = 0;
        }
        public int getCycleIncrement(int desiredCycle){ //Used for cycle based carosel differences for movement

            int step = 0;

            int N = 3;

            int diff = desiredCycle - currentCycle;

            int modDiff = ((diff % N) + N) % N;

            if (modDiff > N/ 2){
                step = (modDiff - N) * 120;
            }
            else{
                step = (modDiff) * 120;
            }

            RobotConstantsV2.CAROSEL_GLOBAL_INCREMENT += step;

            return RobotConstantsV2.CAROSEL_GLOBAL_INCREMENT;
        }
        private int getNearestUselessArtifact(){

            ArrayList<String> tempPattern = new ArrayList<String>();

            for (String p:pattern){
                tempPattern.add(p);
            }

            for (String i: inventory){
                for (String p : pattern){
                    if (i.equals(p)){
                        tempPattern.remove(p);
                    }
                }
            }

            if (tempPattern.isEmpty()){
                return -1;
            }

            return getInvPosition(tempPattern.get(0));

        }
        private int getNearestArtifact(){

            int count = 0;

            for (String i:inventory){
                if (i.equals("Green") || i.equals("Purple")){
                    return count;
                }
                count ++;
            }
            return count;
        }

        //TODO not optimized yet for patterns (if had GPE, but pattern PPG, then would shoot GP (not PG for max points)
        private int getNextCycle(){
            if (currentCycle == 0){
                return 1;
            }

            else if (currentCycle == 1){
                return 2;
            }

            else if (currentCycle == 2){
                return 0;
            }

            return -1;
        } //TODO can replace this with different method, but it works
        public int getCurrentCycle(){
            return currentCycle;
        }


        /** Intake Detection */
        public float[] getHSVFront(){

            float[] hsv = new float[]{0F,0F,0F};

            NormalizedRGBA colors = colorSensorFront.getNormalizedColors();

            Color.RGBToHSV((int) (colors.red *255), (int) (colors.green *255), (int) (colors.blue * 255), hsv);
            return hsv;
        }
        public float[] getHSVBack(){

            float[] hsv = new float[]{0F,0F,0F};

            NormalizedRGBA colors = colorSensorBack.getNormalizedColors();

            Color.RGBToHSV((int)(colors.red *255), (int)(colors.green *255), (int)(colors.blue * 255), hsv);
            return hsv;
        }
        public double getRangerDistance(){
            return (rangerSensor.getVoltage() * RobotConstantsV2.RANGER_EQU_SLOPE + RobotConstantsV2.RANGER_EQU_Y_INT) * RobotConstantsV2.IN_TO_CM;
        }
        public boolean detectedArtifact(){

            if (!isCaroselInPlace()) return false;

            return (getRangerDistance() > RobotConstantsV2.RANGER_DETECTION_MIN_THRESHOLD && getRangerDistance() < RobotConstantsV2.RANGER_DETECTION_MAX_THRESHOLD);
        }

        public boolean isShotConfirmed(){

            if (!isCaroselInPlace()) return false;

            return getRangerDistance() > RobotConstantsV2.RANGER_DETECTION_CONFIRM_SHOT;
        }


        public String getColorBack() { //TODO here issue with megging
//            if (detectedArtifactBack()) {
//                float[] HSV = getHSVBack();
//
//                if (HSV[0] < RobotConstantsV2.MIDDLE_H && HSV[1] >= RobotConstantsV2.MIDDLE_S){
//                    return "Green";
//                }
//
//                else{
//                    return "Purple";
//                }
//            }
            float[] HSV = getHSVBack();

            //&& HSV[2] >= RobotConstantsV2.MIN_V
            if (detectedArtifactBack()){
                if (HSV[0] < RobotConstantsV2.MIDDLE_H && HSV[1] >= RobotConstantsV2.MIN_S){return "Green";}
                else if (HSV[0] >= RobotConstantsV2.MIDDLE_H && HSV[1] >= RobotConstantsV2.MIN_S){return "Purple";}
            }
            return "Empty";
        }

        public boolean detectedArtifactFront(){
            return colorSensorFrontDist.getDistance(DistanceUnit.CM) < RobotConstantsV2.COLOR_SENSOR_DIST_THRESHOLD_FRONT && isCaroselInPlace();

            //float[] HSV = getHSVFront();
            //return HSV[1] > RobotConstantsV2.MIN_S && HSV[2] > RobotConstantsV2.MIN_V;
        }
        public boolean detectedArtifactBack(){
            return colorSensorBackDist.getDistance(DistanceUnit.CM) < RobotConstantsV2.COLOR_SENSOR_DIST_THRESHOLD_BACK && isCaroselInPlace();

            //float[] HSV = getHSVBack();
            //return HSV[1] > RobotConstantsV2.MIN_S && HSV[2] > RobotConstantsV2.MIN_V;
        }
        public String getColorFront(){

            //TODO old method

//            if (detectedArtifactFront()) {
//                float[] HSV = getHSVFront();
//
//                if (HSV[0] < RobotConstantsV2.MIDDLE_H && HSV[1] >= RobotConstantsV2.MIN_S){
//                    return "Green";
//                }
//
//                else{
//                    return "Purple";
//                }
//            }

            float[] HSV = getHSVFront();

            if (detectedArtifactFront()){
                if (HSV[0] < RobotConstantsV2.MIDDLE_H && HSV[1] >= RobotConstantsV2.MIN_S){return "Green";}
                else if (HSV[0] >= RobotConstantsV2.MIDDLE_H && HSV[1] >= RobotConstantsV2.MIN_S){return "Purple";}
            }

            return "Empty";
        }


        /** Indicator Lights */
        public void indicatorsInInit(){
            indicatorOne.setPosition(RobotConstantsV2.INDICATOR_ORANGE);
            indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_ORANGE);
        }
        public void updateIndicators(String mode, double disp, LimeLightVision limelight){

            switch (mode){
                case("auto"):

//                    if (!limelight.getResults().isValid()){
//                        disp = RobotConstantsV2.CLOSE_BALL_DISTANCE;
//                    }

                    disp = RobotConstantsV2.CLOSE_BALL_DISTANCE;
//                    if (!limelight.canSeeSomeAT() || !turret.isToggleTurretAim() || Math.abs(turret.getTPSError(disp)) > turret.getTPS(disp) * RobotConstantsV2.SHOOTER_SPEED_THRESHOLD){
//                        indicatorOne.setPosition(RobotConstantsV2.INDICATOR_RED);
//                    }
                    if (!turret.isToggleTurretAim() || !getTurret().isUpToSpeed(disp)){
                        indicatorOne.setPosition(RobotConstantsV2.INDICATOR_RED);
                    }
                    else{
                        indicatorOne.setPosition(RobotConstantsV2.INDICATOR_BLUE);
                    }

                    //Current Color

                    if (inventory[currentCycle].equals("Purple")){
                        indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_PURPLE);
                    }
                    else if (inventory[currentCycle].equals("Green")){
                        indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_GREEN);
                    }
                    else{
                        if (getColorBack().equals("Purple")){
                            indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_PURPLE);
                        }
                        else if (getColorBack().equals("Green")){
                            indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_GREEN);
                        }
                        else{
                            indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_YELLOW);
                        }
                    }

                    break;

                case("manual"):

                    if (turret.isFarToggled()){
//                        telemetry.addData("Distance: ", disp);
//                        telemetry.addData("Stupid TPS Error: ", turret.getTPSError(disp));
//                        telemetry.addData("Attempted TPS: ", turret.getTPS(disp));
//                        telemetry.addData("Threshold Max: ", turret.getTPS(disp) * RobotConstantsV2.SHOOTER_SPEED_THRESHOLD);

                        if (!getTurret().isUpToSpeed(disp)){
                            indicatorOne.setPosition(RobotConstantsV2.INDICATOR_RED);
                        }
                        else{
                            indicatorOne.setPosition(RobotConstantsV2.INDICATOR_BLUE);
                        }
                    }
                    else{
                        disp = RobotConstantsV2.CLOSE_BALL_DISTANCE;

                        if (!getTurret().isUpToSpeed(disp)){
                            indicatorOne.setPosition(RobotConstantsV2.INDICATOR_RED);
                        }
                        else{
                            indicatorOne.setPosition(RobotConstantsV2.INDICATOR_BLUE);
                        }
                    }

                    //Current Color (Binary)
                    if (detectedArtifact()){
                        indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_BLUE);
                    }
                    else{
                        indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_YELLOW);
                    }

                    break;

                case("humanIntake"):

                    indicatorOne.setPosition(RobotConstantsV2.INDICATOR_ORANGE); //Shows that Human Player Mode

                    if (!isEmptySpot()){
                        indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_ORANGE);
                    }
                    else if ((!detectedArtifact() || inventory[currentCycle].equals("Empty")) && isCaroselInPlace()){
                        indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_GREEN);
                    }
                    else{
                        indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_YELLOW);
                    }

                    break;

                default:
                    //UH OH SOME ERROR HERE
                    indicatorOne.setPosition(RobotConstantsV2.INDICATOR_RED);
                    indicatorOne.setPosition(RobotConstantsV2.INDICATOR_RED);

                    break;

            }

        }

        /** Final Updates */
        public void telemetryCarosel(){

            telemetry.addLine("------------------------------------\n");

            telemetry.addLine("Carosel \n");

            telemetry.addData("Detected ART: ", detectedArtifact());
            //telemetry.addData("Ranger Dist: ", getRangerDistance());
            //telemetry.addData("Analog Ranger:", rangerSensor.getVoltage());
            telemetry.addLine(String.format("Inventory: (%s, %s, %s)", inventory[0], inventory[1], inventory[2]));
            telemetry.addLine(String.format("Motif: (%s, %s, %s)", pattern[0], pattern[1], pattern[2]));
//            float[] HSVFront = getHSVFront();
//            telemetry.addLine(String.format("HSV Front: (%.5f, %.5f, %.5f)", HSVFront[0], HSVFront[1], HSVFront[2]));
//            float[] HSVBack = getHSVBack();
//            telemetry.addLine(String.format("HSV Back: (%.5f, %.5f, %.5f)", HSVBack[0], HSVBack[1], HSVBack[2]));

            telemetry.addData("Current Cycle: ", currentCycle);
//            telemetry.addData("Distance Front: ", colorSensorFrontDist.getDistance(DistanceUnit.CM));
//            telemetry.addData("Color Front: ", getColorFront());
//            telemetry.addData("Distance Back: ", colorSensorBackDist.getDistance(DistanceUnit.CM));
//            telemetry.addData("Color Back: ", getColorBack());
            //telemetry.addData("IN Place: ", isCaroselInPlace());
//            telemetry.addData("Detected Back: ", detectedArtifactBack());
////            telemetry.addData("Detected Front: ", detectedArtifactFront());
//
//            telemetry.addData("Global Position: ", globalCaroselPosition);
//            telemetry.addData("Inventory Current: ", inventory[currentCycle]);
            telemetry.addData("Has Pattern in Inventory: ", hasPattern());
//            telemetry.addData("Voltage: ", caroselAnalog.getVoltage());

            telemetry.addLine("\n------------------------------------\n");
        }
    }
}