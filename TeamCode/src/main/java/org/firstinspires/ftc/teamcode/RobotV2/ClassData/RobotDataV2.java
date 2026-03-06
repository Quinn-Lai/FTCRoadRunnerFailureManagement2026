package org.firstinspires.ftc.teamcode.RobotV2.ClassData;

import android.graphics.Color;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

        RobotConstantsV2.CAROSEL_DETECTED_ARTIFACT_DELAY = RobotConstantsV2.CAROSEL_DETECTED_ARTIFACT_DELAY_TELE;
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

    public class DriveTrain extends PIDControl{

        /** Drive Train */
        private DcMotor LFmotor;
        private DcMotor RFmotor;
        private DcMotor LBmotor;
        private DcMotor RBmotor;

        /** Driver Status */
        private boolean robotCentric;
        private Gamepad driver;

        /** Modes */
        private String currentMode;
        private String currentSubMode;
        private double lastErrorHeading;

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

            lastErrorHeading = 0;
        }

        //----------------------------------------

        /** Main Modes */
        public void forceModeAuto(){
            currentMode = RobotConstantsV2.mainModes[0];
        }

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
        public void setCurrentMode(String mode){
            currentMode = mode;
        }


        /** Sub Modes */
        public String getCurrentSubMode(){
            return currentSubMode;
        }
        public void setCurrentSubMode(String mode){
            currentSubMode = mode;
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
                                getCarosel().startFailsafeSubmode();

                                if (getCarosel().isFailsafeSubmode()){
                                    getCarosel().endFailsafeSubmode();
                                    getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
                                    getCarosel().activateTransferInProg();
                                }
                            }

                            break;

                        case ("transfer"):

                            getCarosel().subModeTransferStartTimer();
                            getCarosel().incrementRapidFireCurrentShotCount();
                            getCarosel().activateCycleInProg();
                            getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);

                            break;

                        default:
                            break;

                    }

                    break;

                case ("sortedFire"):

                    if (getCarosel().getSortedFireCurrentShotCount() >= getCarosel().getMaxShots() && !getCarosel().isTransferCooldownActive()) {
                        getDriveTrain().endSubMode();
                        getCarosel().cycleOrigin();
                        break;
                    }

                    switch (getCarosel().getCurrentSubModeQueue()) {
                        case ("cycle"):

                            if (!getCarosel().isTransferCooldownActive()) {

                                getCarosel().cycleSortedFire(getCarosel().getSortedFireCurrentShotCount());
                                getCarosel().startFailsafeSubmode();

                                if (getCarosel().isCaroselInPlace() || getCarosel().isFailsafeSubmode()) {
                                    getCarosel().endFailsafeSubmode();
                                    getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
                                    getCarosel().activateTransferInProg();
                                }
                            }

                            break;

                        case ("transfer"):

                            getCarosel().subModeTransferStartTimer();
                            getCarosel().incrementSortedFireCurrentShotCount();
                            getCarosel().activatePatternInProg();
                            getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);

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
            //getTurret().aimBall(distance);

            if (getCarosel().getSortedFireCurrentShotCount() >= getCarosel().getMaxShots() && !getCarosel().isTransferCooldownActive()) {
                getCarosel().transferReceiveTimer();
                getCarosel().resetSortedFireCurrentShotCount();
                getDriveTrain().endSubMode();
                //getCarosel().cycleOrigin();
                return false;
            }

            switch (getCarosel().getCurrentSubModeQueue()) {
                case ("cycle"):

                    if (!getCarosel().isTransferCooldownActive()) {

                        getCarosel().cycleSortedFire(getCarosel().getSortedFireCurrentShotCount());
                        getCarosel().startFailsafeSubmode();

                        if (getCarosel().isCaroselInPlace() || getCarosel().isFailsafeSubmode()) {
                            getCarosel().endFailsafeSubmode();
                            getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
                            getCarosel().activateTransferInProg();
                        }
                    }

                    break;

                case ("transfer"):
                    getCarosel().subModeTransferStartTimer();
                    getCarosel().incrementSortedFireCurrentShotCount();
                    getCarosel().activatePatternInProg();
                    getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);

                    break;

                default:
                    break;
            }

            return true;
        }
        public boolean executePatternFireAutoFar(double distance){
            getCarosel().updateCaroselEncoder();
            //getTurret().aimBall(distance);

            if (getCarosel().getSortedFireCurrentShotCount() >= getCarosel().getMaxShots() && !getCarosel().isTransferCooldownActive()) {
                getCarosel().transferReceiveTimer();
                getCarosel().resetSortedFireCurrentShotCount();
                getDriveTrain().endSubMode();
                //getCarosel().cycleOrigin();
                return false;
            }

            switch (getCarosel().getCurrentSubModeQueue()) {
                case ("cycle"):

                    if (!getCarosel().isTransferCooldownActive()) {

                        getCarosel().cycleSortedFire(getCarosel().getSortedFireCurrentShotCount());
                        getCarosel().startFailsafeSubmode();

                        if (getCarosel().isCaroselInPlace() || getCarosel().isFailsafeSubmode()) {
                            getCarosel().endFailsafeSubmode();
                            getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
                            getCarosel().activateTransferInProg();
                        }
                    }

                    break;

                case ("transfer"):

                    boolean status = getTurret().isUpToSpeed(RobotConstantsV2.FAR_BALL_DISTANCE);
                    if (status){
                        getCarosel().subModeTransferStartTimer();
                        getCarosel().incrementSortedFireCurrentShotCount();
                        getCarosel().activatePatternInProg();getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
                    }
                    break;

                default:
                    break;
            }

            return true;
        }
        public boolean executeRapidFireAuto(double distance){
            getCarosel().updateCaroselEncoder();
            //getTurret().aimBall(distance);

            if (getCarosel().getRapidFireCurrentShotCount() >= RobotConstantsV2.RAPID_FIRE_MAX_SHOTS && !getCarosel().isTransferCooldownActive()) {
                getCarosel().transferReceiveTimer();
                getCarosel().resetRapidFireCurrentShotCount();
                getDriveTrain().endSubMode();
                //getCarosel().cycleOrigin();
                return false;
            }

            switch (getCarosel().getCurrentSubModeQueue()){
                case ("cycle"):

                    if (!getCarosel().isTransferCooldownActive()){
                        getCarosel().cycleRapidFire();
                        getCarosel().startFailsafeSubmode();

                        if (getCarosel().isFailsafeSubmode()){
                            getCarosel().endFailsafeSubmode();
                            getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
                            getCarosel().activateTransferInProg();
                        }
                    }

                    break;

                case ("transfer"):

                    getCarosel().subModeTransferStartTimer();
                    getCarosel().incrementRapidFireCurrentShotCount();
                    getCarosel().activateCycleInProg();
                    getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);


                default:
                    break;

            }

            return true;
        }


        /** Movement */
        public void switchRobotCentric(){
            if (robotCentric) robotCentric = false;
            else robotCentric = true;
        }

        public void forceRobotCentricOn(){
            robotCentric = true;
        }

        public void forceRobotCentricOff(){
            robotCentric = false;
        }

        public void powerMotorSafe(DcMotor motor, double targetPower){
            final double SLEW = 0.2;
            double currentPower = motor.getPower();

            double desiredChange = targetPower - currentPower;
            double limitedChange = Math.max(-SLEW, Math.min(desiredChange, SLEW));

            motor.setPower(currentPower += limitedChange);
        }
        public void omniDrive(RoadRunnerDataV2 rrData, String alliance){

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

                double desiredHeading = rrData.getYawLocalization(alliance, 0);
                double currentHeading = rrData.getYaw360();

                if (Math.abs(z) < 0.1) {

                    z = PIDHeading(currentHeading,desiredHeading);

                    //telemetry.addData("Z Power Reduction: ", z);

                }
                else{
                    z *= -1;
                }

                rrData.getDrive().setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                y,
                                -x
                        ),
                        z
                ));
            }
        }
        public void killAllWheels(){
            LFmotor.setPower(0);
            LBmotor.setPower(0);
            RFmotor.setPower(0);
            RBmotor.setPower(0);
        }

        /** Gamepad */
        public void setGamepad(Gamepad driver) {
            this.driver = driver;
        }
        public Gamepad getGampad(){
            return driver;
        }
        public void rumbleOne(){
            driver.rumble(1,0,200);
        }

        public void rumbleTwo(){
            driver.rumble(1,0,500);
        }
        public void rumbleStrong(){
            driver.rumble(1);
        }

        /** Final Updates */
        public void updateModeColor(){
            if (currentMode.equals(RobotConstantsV2.mainModes[0])){

                if (!turret.isAutoSetPosActive()){
                    driver.setLedColor(1,0,0,Gamepad.LED_DURATION_CONTINUOUS);
                }

                else{
                    if (turret.isFarToggled()){
                        driver.setLedColor( 0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
                    }
                    else{
                        driver.setLedColor( 0, 0.2, 0, Gamepad.LED_DURATION_CONTINUOUS);
                    }
                }
            }
            else if (currentMode.equals(RobotConstantsV2.mainModes[1])){
                if (turret.isFarToggled()){
                    driver.setLedColor( 0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
                }
                else{
                    driver.setLedColor( 0, 0, 0.2, Gamepad.LED_DURATION_CONTINUOUS);
                }
            }
            else{
                driver.setLedColor( 1, 0.67, 0, Gamepad.LED_DURATION_CONTINUOUS);
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
        private ElapsedTime PIDTimer = new ElapsedTime();
        private double integralSum = 0;
        private double lastError = 0;

        /** PID for Carosel */
//        private double kPC = 0.0019;
//        private double kIC = 0.000021; //0.000005
//        private double kDC = 0.013; //0.13
        private ElapsedTime PIDTimerHeading = new ElapsedTime();
        private double integralSumHeading = 0;
        private double lastErrorHeading = 0;

        //----------------------------------------

        /** Get Constants for Shooter */
        public String getPIDCos(){
            return String.format("kP: %f, kI: %f, kD: %f", RobotConstantsV2.kPHeading, RobotConstantsV2.kIHeading, RobotConstantsV2.kDHeading);
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

            output = (error * RobotConstantsV2.kP) + (derivate * RobotConstantsV2.kD) + (integralSum * RobotConstantsV2.kI) + (desired * RobotConstantsV2.kF);

            return output;
        }
        public double PIDHeading(double current, double desired) {

            double currentTimeC = PIDTimer.seconds();

            double outputC = 0;

            double errorC = desired - current;

            integralSumHeading += errorC * currentTimeC;

            double derivateC = (errorC - lastErrorHeading) / currentTimeC;

            lastError = errorC;

            PIDTimerHeading.reset();

            double positionalCalc = errorC * RobotConstantsV2.kPHeading;
            double integralCalc = integralSumHeading * RobotConstantsV2.kIHeading;
            double derivativeCalc = derivateC * RobotConstantsV2.kDHeading;

            outputC = (positionalCalc) + (derivativeCalc) + (integralCalc);

            if (outputC > 0.5) outputC = 0.5;
            else if (outputC < -0.5) outputC = -0.5;

            return outputC;
        }

    }

    //----------------------------------------

    public class Turret extends PIDControl{

        /** Parts */
        public DcMotorEx shooterMotorOne;
        public DcMotorEx shooterMotorTwo;
        private Servo shooterServo;

        /** Physics Parameters */
        private final double a = -9.8;
        public double heightOfLauncher = 0.3556;
        private final double height = RobotConstantsV2.HEIGHT_TO_AIM - heightOfLauncher;
        private final double vY = Math.sqrt(2 * -a * height);
        private double vX = 0;
        private double vX0 = 0; //initial vX of the robot at an instant in time

        /** Fly Wheel */
        private final double shooterWheelRadius = 0.096/2; //Meters
        private final double w = 28.0; //Encoder Resolution for 6K RPM

        /** Toggle Turret */
        private boolean toggleTurretAim;
        private boolean autoSetPosActive;
        private boolean isFar;

        //----------------------------------------

        /** Constructor */
        private Turret(){

            /** Hardware Init */
            shooterMotorOne = (DcMotorEx)hardwareMap.get(DcMotor.class,"shooterMotorOne");
            shooterMotorTwo = (DcMotorEx)hardwareMap.get(DcMotor.class,"shooterMotorTwo");
            shooterServo = hardwareMap.get(Servo.class,"shooterServo");

            //shooterMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooterMotorOne.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotorOne.setDirection(DcMotor.Direction.REVERSE);

            //shooterMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooterMotorTwo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotorTwo.setDirection(DcMotor.Direction.FORWARD);

            /** Mode Init */
            toggleTurretAim = false;
            isFar = false;
            autoSetPosActive = false;
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
        public void updateNormX(RoadRunnerDataV2 roadRunnerDataV2){
            //double initalVel = roadRunnerDataV2.getNormalVelocity(limelight.getAlliance());
            vX0 = roadRunnerDataV2.getVelocityX();
        }

        public void forceZeroVX0(){
            vX0 = 0;
        }

        private double getVx(double disp){
            return ((disp / getTimeExtrema()) - vX0);
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
        private double getW(){
            return w;
        }
        public double getTPS(double disp){ //From Radians per Second

            //TODO Turn into an equation relating distance to drag multiplier
            double TPSfound = ((getRadiansPerSecond(disp) * getW()) / (2 * Math.PI)) * getDragMultiplier(disp);

            if (TPSfound > RobotConstantsV2.MAX_TURRET_TPS) TPSfound = RobotConstantsV2.MAX_TURRET_TPS;

            return TPSfound;
        }

        private double getDragMultiplier(double disp){
            //return 0.0848964 * disp * disp - 0.440512 * disp + 2.67034;
            return 0.0585876 * disp * disp - 0.336472 * disp + 2.38658;
        }

        private double getCurrentLinearVelocity(){
            return shooterMotorOne.getVelocity() * ((2 * Math.PI) / w) * shooterWheelRadius;
        }

        /** Turret Modes */
        public void switchAutoSetPos(){
            if (autoSetPosActive) autoSetPosActive = false;
            else autoSetPosActive = true;
        }
        public boolean isAutoSetPosActive(){
            return autoSetPosActive;
        }
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
            shooterMotorOne.setDirection(DcMotorEx.Direction.FORWARD);
            shooterMotorTwo.setDirection(DcMotorEx.Direction.REVERSE);
            powerShooterMotor(RobotConstantsV2.HUMAN_INTAKE_SPEED);
        }
        public void deactivateHumanIntakeMode(){
            shooterMotorOne.setDirection(DcMotorEx.Direction.REVERSE);
            shooterMotorTwo.setDirection(DcMotorEx.Direction.FORWARD);
        }

        /** Shooter Power */
        public void powerShooterMotor(double TPS){

            if (TPS > RobotConstantsV2.MAX_TURRET_TPS) TPS = RobotConstantsV2.MAX_TURRET_TPS;


            double current = shooterMotorOne.getVelocity();
            //shooterMotor.setVelocity(PIDShooter(current,TPS));
            double motorPower = PIDShooter(current,TPS);
            shooterMotorOne.setPower(motorPower);
            shooterMotorTwo.setPower(motorPower);
            //telemetry.addData("TPS NOW", TPS);
            //telemetry.addData("TPS Current: ", current);

        }
        public void killShooter(){
            //shooterMotor.setVelocity(RobotConstantsV2.KILL_SHOOTER_SPEED);
            shooterMotorOne.setPower(0);
            shooterMotorTwo.setPower(0);
        }

        /** Hood */
        private double convertDegToServo(double angle){

            //telemetry.addData("Angle: ", angle);
            if (angle > RobotConstantsV2.MAX_HOOD_ANGLE) return RobotConstantsV2.MAX_HOOD_ANGLE_POS;
            else if (angle < RobotConstantsV2.MIN_HOOD_ANGLE) return RobotConstantsV2.MIN_HOOD_ANGLE_POS;

            //Failsafe

            //double pos = (double) Math.round((0.00518135 * angle + 0.484197) * 1000) / 1000;
            double pos = (double) Math.round((-0.00000817122 * angle * angle + 0.0076676 * angle + 0.39316) * 1000) / 1000;

            if (pos < RobotConstantsV2.MIN_HOOD_ANGLE_POS) pos = RobotConstantsV2.MIN_HOOD_ANGLE_POS;
            else if (pos > RobotConstantsV2.MAX_HOOD_ANGLE_POS) pos = RobotConstantsV2.MAX_HOOD_ANGLE_POS;;

            return pos;
        }
        public void angleRobot(double disp){
            double desiredAngle = getAngleTotal(disp) + RobotConstantsV2.ANGLE_BONUS;
            shooterServo.setPosition(convertDegToServo(desiredAngle));
        }
        public void angleRobotVelControl(double disp){

            double vY = getVy();
            double lV = getCurrentLinearVelocity()/getDragMultiplier(disp);

            double desiredAngle = 0;

            if (lV >= vY){
                desiredAngle = Math.toDegrees(Math.asin(vY * RobotConstantsV2.VERTICAL_MULTIPLIER/lV));
            }

            shooterServo.setPosition(convertDegToServo(desiredAngle));
        }
        public void telemetryDebug(RoadRunnerDataV2 roadRunnerDataV2, LimeLightVision limeLightVision){

            double disp = limeLightVision.getDisp();
            double lV = getCurrentLinearVelocity()/getDragMultiplier(disp);

            telemetry.addLine(" ------------------ Test 1 ---------------- \n");

            telemetry.addData("Voltage: ", getCarosel().caroselAnalog.getVoltage());
            telemetry.addData("Degrees: ", getCarosel().getCaroselDegrees());
            telemetry.addData("Displacement (m): ", disp);
            telemetry.addData("Localizer  Displacement (m): ", roadRunnerDataV2.getDispLocalization(limeLightVision.getAlliance()));
            telemetry.addData("Adjusted Hood Angle: ", Math.toDegrees(Math.asin(vY * RobotConstantsV2.VERTICAL_MULTIPLIER/lV)));
            telemetry.addData("True Desired Hood Angle: ", getAngleTotal(disp));
            telemetry.addData("True Desired TPS: ", getTPS(disp));
            telemetry.addData("Current Linear Velocity: ", getCurrentLinearVelocity());
            telemetry.addData("Normalized TPS: ", getTPS(disp)/getDragMultiplier(disp));
            telemetry.addData("Normalized Linear Velocity: ", lV);
            telemetry.addData("Y-Component: ", getVy());

            telemetry.addLine("\n ------------------ Test 2 ---------------- \n");

            telemetry.addData("Color Front: ", getCarosel().getColorFront());
            telemetry.addData("Color Back: ", getCarosel().getColorBack());

            telemetry.addLine("HSV Front: " + getCarosel().getHSVFront()[0] + ", " + getCarosel().getHSVFront()[1] + ", " + getCarosel().getHSVFront()[2]);
            telemetry.addLine("HSV Back: " + getCarosel().getHSVBack()[0] + ", " + getCarosel().getHSVBack()[1] + ", " + getCarosel().getHSVBack()[2]);

            telemetry.addData("X Velocity: ", roadRunnerDataV2.getVelocityX());
            telemetry.addData("Y Velocity: ", roadRunnerDataV2.getVelocityY());
            telemetry.addData("X Velocity Norm: ", roadRunnerDataV2.getNormXVel());
            telemetry.addData("Y Velocity Norm: ", roadRunnerDataV2.getNormYVel());
            telemetry.addData("Angle Velocity: ", roadRunnerDataV2.getRobotTheta());
            telemetry.addData("Velocity Norm Tower: ", roadRunnerDataV2.getVelTowerSurface(limeLightVision.getAlliance()));
            telemetry.addData("Localizer Yaw: ", roadRunnerDataV2.getYaw());
            telemetry.addData("Limelight MT1 Yaw: ", limeLightVision.getYawMT1());
            telemetry.addData("Limelight MT2 Yaw: ", limeLightVision.getResults().getBotpose_MT2().getOrientation().getYaw(AngleUnit.DEGREES));
        }

        /** Final Shot */
        public void aimBall(double disp){

            if (disp > RobotConstantsV2.DIST_CHANGE_THRESHOLD){
                angleRobotVelControl(disp);
            }
            else{
                angleRobot(disp);
            }
            double TPS = getTPS(disp);
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

            telemetry.addData("Current Shooter Motor TPS One: ", shooterMotorOne.getVelocity());
            telemetry.addData("Current Shooter Motor TPS Two: ", shooterMotorTwo.getVelocity());

            telemetry.addData("Desired TPS: ", getTPS(disp));
            telemetry.addData("TPS Error:", getTPSError(disp));
            telemetry.addData("Displacement (m): ", disp);
            telemetry.addData("Angle: ", getAngleTotal(disp));

        }
        /** Tune PID Data*/

        public void telemetryPID(double disp){

            telemetry.addData("PID ADJUSTED 1: ", PIDShooter(shooterMotorOne.getVelocity(), getTPS(disp)));
            telemetry.addData("PID ADJUSTED 2 (COSMETIC BOTH USING 1): ", PIDShooter(shooterMotorTwo.getVelocity(), getTPS(disp)));
            telemetry.addData("PID Co: ", getPIDCos());
            telemetry.addData("True TPS 1: ", shooterMotorOne.getVelocity());
            telemetry.addData("True TPS 2: ", shooterMotorTwo.getVelocity());
            telemetry.addData("Desired Ticks Per Second: ", getTPS(disp));
            telemetry.addData("TPS Error:", shooterMotorOne.getVelocity() - getTPS(disp));
            telemetry.addData("TPS Error:", shooterMotorTwo.getVelocity() - getTPS(disp));

        }
        public void telemetryTurretBasic(){
            telemetry.addData("Turret On: ", toggleTurretAim);
            telemetry.addData("Far Shot: ", !isFar);
        }
        public double getTPSError(double disp){
            return shooterMotorOne.getVelocity() - getTPS(disp);
        }
        public double getTPSErrorManual(double TPS){
            return shooterMotorOne.getVelocity() - TPS;
        }
        public boolean isUpToSpeed(double distance){
            //Current error less than allowed error
//            telemetry.addData("TPS Error: ", getTPSError(distance));
//            telemetry.addData("TPS: ", getTurret().getTPS(distance));
//            telemetry.addData("Current TPS: ", shooterMotor.getVelocity());

            //TODO: TPS Both
            return Math.abs(getTPSError(distance)) < getTurret().getTPS(distance) * RobotConstantsV2.SHOOTER_SPEED_THRESHOLD;
        }
    }

    //----------------------------------------

    public class Carosel extends PIDControl{

        /** Hardware */
        private DcMotorEx intakeMotor;
        private Servo transferServo;
        private Servo caroselServoPosL;
        private Servo caroselServoPosR;
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
        private String inventoryCache;

        /** Intake  */
        private boolean intakeMotorOn;
        private boolean ejectActive;

        /** Transfer */
        private boolean transferUp;
        private ElapsedTime transferCooldown;
        private boolean transferCooldownActive;
        private boolean ignoreArtifactDetected;

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
        private ElapsedTime ignoreArtifactDetectedTimer;

        /** PID */
        private double lastCaroselPosition;
        private double globalCaroselPosition;

        //----------------------------------------

        /** Constructor */
        public Carosel(){

            /** Hardware Init */

            caroselServoPosL = hardwareMap.get(Servo.class,"caroselServoL"); //TODO temp
            caroselServoPosR = hardwareMap.get(Servo.class,"caroselServoR"); //TODO temp
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
            inventoryCache = "Empty";

            /** Variable Init */

            cycleInProg = false;
            patternInProg = false;
            transferInProg = false;
            intakeMotorOn = false;
            transferUp = false;
            transferCooldownActive = false;
            shotSucceed = false;
            failsafeSubmode = false;
            ejectActive = false;
            ignoreArtifactDetected = false;

            /** Timers */
            failsafeTimer = new ElapsedTime();
            transferCooldown = new ElapsedTime();
            ignoreArtifactDetectedTimer = new ElapsedTime();

            /** Sub Modes */
            rapidFireCurrentShotCount = 0;
            sortedFireCurrentShotCount = 0;
        }

        //----------------------------------------

        public void resetBooleans(){
            cycleInProg = false;
            patternInProg = false;
            transferInProg = false;
            transferCooldownActive = false;
            shotSucceed = false;
            failsafeSubmode = false;
            ignoreArtifactDetected = false;
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
                getDriveTrain().rumbleOne();
                intakeMotor.setVelocity(RobotConstantsV2.INTAKE_ON);
                intakeMotorOn = true;
            }
            else{
                getDriveTrain().rumbleTwo();
                intakeMotor.setVelocity(RobotConstantsV2.INTAKE_OFF);
                intakeMotorOn = false;
            }
        }
        public void switchReverseIntake(){
            intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
            if (!intakeMotorOn){
                getDriveTrain().rumbleOne();
                intakeMotor.setVelocity(RobotConstantsV2.INTAKE_IN_EJECT);
                intakeMotorOn = true;
            }
            else{
                getDriveTrain().rumbleTwo();
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
            //transferCooldownActive = false;
        }
        public void transferStartTimer(){
            if (turret.isToggleTurretAim() && !driveTrain.getCurrentMode().equals(RobotConstantsV2.mainModes[2])){
                transferCooldownActive = true;
                forceTransferUp();
                transferCooldown.reset();
            }
        } //TODO simplify
        public void transferReceiveTimer(){
            if (transferCooldownActive && transferCooldown.milliseconds() > RobotConstantsV2.COOLDOWN_SHOT_UP){
                forceTransferDown();
                if (transferCooldown.milliseconds() > (double)RobotConstantsV2.COOLDOWN_SHOT_DOWN + RobotConstantsV2.COOLDOWN_SHOT_DOWN) transferCooldownActive = false;
            }
        }
        public void activateTransferInProg(){
            transferInProg = true;
        }
        public void subModeTransferStartTimer(){
            if (transferInProg){ //
                transferInProg = false;
                transferCooldownActive = true;
                forceTransferUp();
                transferCooldown.reset();
            }
        }
        public boolean isTransferCooldownActive(){
            return transferCooldownActive;
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
            if (!failsafeSubmode){
                failsafeSubmode = true;
                failsafeTimer.reset();
            }
        }
        public boolean isFailsafeSubmode(){
            //Cosmetic rn

            double timer = RobotConstantsV2.FAILSAFE_SUBMODE_TIMER;

            if (getDriveTrain().getCurrentSubMode().equals(RobotConstantsV2.subModes[1])){
                timer = RobotConstantsV2.FAILSAFE_SUBMODE_TIMER_LONG;
            }

            if (failsafeSubmode && failsafeTimer.milliseconds() > timer){
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

            String[] tempArrayInventory = Arrays.copyOf(inventory, inventory.length);
            String[] tempArrayPattern = Arrays.copyOf(pattern, pattern.length);

            Arrays.sort(tempArrayInventory);
            Arrays.sort(tempArrayPattern);

            return Arrays.equals(tempArrayInventory, tempArrayPattern);
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

        public void updateInventoryCache(){
            if (inventory[currentCycle].equals("Empty")){
                if (inventoryCache.equals("Empty")){
                    inventoryCache = getColorFront();
                }

                if (inventoryCache.equals("Empty")){
                    inventoryCache = getColorBack();
                }
            }
        }

        private void emptyInventoryCache(){
            inventoryCache = "Empty";
        }

        public void autoIntakeCycle(){
            //telemetry.addData("Next Empty Spot: ", getEmptySpot());
            updateCaroselEncoder();
            updateInventoryCache();

            if (currentCycle == 2 && ignoreArtifactDetected && ignoreArtifactDetectedTimer.milliseconds() > RobotConstantsV2.CAROSEL_DETECTED_ARTIFACT_DELAY_LAST_SLOT){
                ignoreArtifactDetected = false;
            }

            else if (ignoreArtifactDetected && ignoreArtifactDetectedTimer.milliseconds() > RobotConstantsV2.CAROSEL_DETECTED_ARTIFACT_DELAY ){
                ignoreArtifactDetected = false;
            }
//!ignoreArtifactDetected
            if (!transferCooldownActive && detectedArtifact() && !ignoreArtifactDetected && isEmptySpot()){
                ignoreArtifactDetected = true;
                ignoreArtifactDetectedTimer.reset();

                inventory[currentCycle] = inventoryCache;
                emptyInventoryCache();

                if (inventory[currentCycle].equals("Empty")){
                    inventory[currentCycle] = getColorBack();
                    if (inventory[currentCycle].equals("Empty")) inventory[currentCycle] = RobotConstantsV2.FAILSAFE_INTAKE; //Failsafe
                }

                if (isEmptySpot()) cycleCarosel(getEmptySpot());
            }
        }
        public void cycleOrigin(){
            cycleCarosel(0);
        }

        public void cycleCarosel(int desiredCycle){
            if (!transferCooldownActive){
                double increm = RobotConstantsV2.caroselPositions[desiredCycle];
                currentCycle = desiredCycle;
                caroselServoPosL.setPosition(increm);
                caroselServoPosR.setPosition(increm);
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

        public boolean isBusy(){
            //double targetPos = -318.91011 * (RobotConstantsV2.caroselPositions[currentCycle]) + 337.27894;
            //double targetPos = -639.15656 * RobotConstantsV2.caroselPositions[currentCycle] + 671.52095;
            double targetPos = -640.89571 * RobotConstantsV2.caroselPositions[currentCycle] + 674.28203;

            return Math.abs(targetPos - globalCaroselPosition) > RobotConstantsV2.CAROSEL_TOLERANCE;
        }

        public boolean isCaroselInPlaceIntake(){
            //double targetPos = -639.15656 * RobotConstantsV2.caroselPositions[currentCycle] + 671.52095;
            double targetPos = -640.89571 * RobotConstantsV2.caroselPositions[currentCycle] + 674.28203;

            telemetry.addData("Target Degree: ", targetPos);
            telemetry.addData("Global Pos: ", globalCaroselPosition);

            return Math.abs(targetPos - globalCaroselPosition) < RobotConstantsV2.CAROSEL_TOLERENCE_INTAKE;
        }

        public boolean isCaroselInPlace(){
            return !isBusy();
        }
        private double getCaroselIncrement(double diff, double current){

            double step = 0;

            double N = 3;

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

        public void updateCaroselEncoder(){
            globalCaroselPosition = caroselAnalog.getVoltage() * RobotConstantsV2.encoderRes;
        }
//        public int getCycleIncrement(int desiredCycle){ //Used for cycle based carosel differences for movement
//
//            int step = 0;
//
//            int N = 3;
//
//            int diff = desiredCycle - currentCycle;
//
//            int modDiff = ((diff % N) + N) % N;
//
//            if (modDiff > N/ 2){
//                step = (modDiff - N) * 120;
//            }
//            else{
//                step = (modDiff) * 120;
//            }
//
//            RobotConstantsV2.CAROSEL_GLOBAL_INCREMENT += step;
//
//            return RobotConstantsV2.CAROSEL_GLOBAL_INCREMENT;
//        }
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

            //ignoreArtifactDetected
            if (ignoreArtifactDetected) return false;

            return (getRangerDistance() > RobotConstantsV2.RANGER_DETECTION_MIN_THRESHOLD && getRangerDistance() < RobotConstantsV2.RANGER_DETECTION_MAX_THRESHOLD);
        }
        public boolean isShotConfirmed(){

            if (!isCaroselInPlace()) return false;

            return getRangerDistance() > RobotConstantsV2.RANGER_DETECTION_CONFIRM_SHOT;
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
//        public String getColorFront(){
//
//            float[] HSV = getHSVFront();
//
//            if (detectedArtifactFront()){
//                if (HSV[0] < RobotConstantsV2.MIDDLE_H && HSV[1] >= RobotConstantsV2.MIN_S){return "Green";}
//                else if (HSV[0] >= RobotConstantsV2.MIDDLE_H && HSV[1] < RobotConstantsV2.MIN_S){return "Purple";}
//            }
//
//            return "Empty";
//        }
//        public String getColorBack() {
////            if (detectedArtifactBack()) {
////                float[] HSV = getHSVBack();
////
////                if (HSV[0] < RobotConstantsV2.MIDDLE_H && HSV[1] >= RobotConstantsV2.MIDDLE_S){
////                    return "Green";
////                }
////
////                else{
////                    return "Purple";
////                }
////            }
//            float[] HSV = getHSVBack();
//
//            //&& HSV[2] >= RobotConstantsV2.MIN_V
//            if (detectedArtifactBack()){
//                if (HSV[0] < RobotConstantsV2.MIDDLE_H && HSV[1] >= RobotConstantsV2.MIN_S){return "Green";}
//                else if (HSV[0] >= RobotConstantsV2.MIDDLE_H && HSV[1] < RobotConstantsV2.MIN_S){return "Purple";}
//            }
//            return "Empty";
//        }

        public String getColorFront(){

            float[] HSV = getHSVFront();

            if (HSV[0] < RobotConstantsV2.MIDDLE_H && HSV[2] > RobotConstantsV2.MIN_V){return "Green";}
            else if (HSV[0] >= RobotConstantsV2.MIDDLE_H && HSV[2] > RobotConstantsV2.MIN_V){return "Purple";}

            return "Empty";
        }
        public String getColorBack() {
            float[] HSV = getHSVBack();

            if (HSV[0] < RobotConstantsV2.MIDDLE_H && HSV[2] > RobotConstantsV2.MIN_V){return "Green";}
            else if (HSV[0] >= RobotConstantsV2.MIDDLE_H && HSV[2] > RobotConstantsV2.MIN_V){return "Purple";}

            return "Empty";
        }

        public void killColorSensors(){
            colorSensorBack.close();
            colorSensorFront.close();
            colorSensorBackDist.close();
            colorSensorFrontDist.close();
        }

        /** Indicator Lights */
        public void indicatorsInInit(){
            indicatorOne.setPosition(RobotConstantsV2.INDICATOR_ORANGE);
            indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_ORANGE);
        }
        public void updateIndicators(String mode, double disp){

            switch (mode){
                case("auto"):

                    //Auto Aim On
                    if (!turret.isAutoSetPosActive()){
                        if (!turret.isToggleTurretAim() || !getTurret().isUpToSpeed(disp)){
                            indicatorOne.setPosition(RobotConstantsV2.INDICATOR_RED);
                        }
                        else{
                            indicatorOne.setPosition(RobotConstantsV2.INDICATOR_BLUE);
                        }
                    }

                    //Auto Aim Off
                    else{
                        if (turret.isFarToggled()){
                            if (!turret.isToggleTurretAim() || !getTurret().isUpToSpeed(RobotConstantsV2.FAR_BALL_DISTANCE)){
                                indicatorOne.setPosition(RobotConstantsV2.INDICATOR_RED);
                            }
                            else{
                                indicatorOne.setPosition(RobotConstantsV2.INDICATOR_BLUE);
                            }
                        }
                        else{
                            if (!turret.isToggleTurretAim() || !getTurret().isUpToSpeed(RobotConstantsV2.CLOSE_BALL_DISTANCE)){
                                indicatorOne.setPosition(RobotConstantsV2.INDICATOR_RED);
                            }
                            else{
                                indicatorOne.setPosition(RobotConstantsV2.INDICATOR_BLUE);
                            }
                        }
                    }

                    //Current Color
                    if (inventory[currentCycle].equals("Purple")){
                        indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_PURPLE);
                    }
                    else if (inventory[currentCycle].equals("Green")){
                        indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_GREEN);
                    }
                    else{
//                        if (getColorBack().equals("Purple")){
//                            indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_PURPLE);
//                        }
//                        else if (getColorBack().equals("Green")){
//                            indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_GREEN);
//                        }
//                        else{
                            indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_YELLOW);
                        //}
                    }

                    break;

                case("manual"):

                    //Shooter Speed
                    if (turret.isFarToggled()){

                        if (!getTurret().isUpToSpeed(RobotConstantsV2.FAR_BALL_DISTANCE)){
                            indicatorOne.setPosition(RobotConstantsV2.INDICATOR_RED);
                        }
                        else{
                            indicatorOne.setPosition(RobotConstantsV2.INDICATOR_BLUE);
                        }
                    }
                    else{
                        if (!getTurret().isUpToSpeed(RobotConstantsV2.CLOSE_BALL_DISTANCE)){
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
            telemetry.addLine(String.format("Motif: (%s, %s, %s)", pattern [0], pattern[1], pattern[2]));
//            float[] HSVFront = getHSVFront();
//            telemetry.addLine(String.format("HSV Front: (%.5f, %.5f, %.5f)", HSVFront[0], HSVFront[1], HSVFront[2]));
//            float[] HSVBack = getHSVBack();
//            telemetry.addLine(String.format("HSV Back: (%.5f, %.5f, %.5f)", HSVBack[0], HSVBack[1], HSVBack[2]));

            telemetry.addData("Current Cycle: ", currentCycle);
            //telemetry.addData("Distance Front: ", colorSensorFrontDist.getDistance(DistanceUnit.CM));
            //telemetry.addData("Color Front: ", getColorFront());
            //telemetry.addData("Distance Back: ", colorSensorBackDist.getDistance(DistanceUnit.CM));
            //telemetry.addData("Color Back: ", getColorBack());
            telemetry.addData("IN Place: ", isCaroselInPlace());
            telemetry.addData("IN Place Intake: ", isCaroselInPlaceIntake());
//            telemetry.addData("Detected Back: ", detectedArtifactBack());
////            telemetry.addData("Detected Front: ", detectedArtifactFront());
//
//            telemetry.addData("Global Position: ", globalCaroselPosition);
//            telemetry.addData("Inventory Current: ", inventory[currentCycle]);
            telemetry.addData("Has Pattern in Inventory: ", hasPattern());
            telemetry.addData("Voltage: ", caroselAnalog.getVoltage());

            telemetry.addLine("\n------------------------------------\n");
        }
    }
}