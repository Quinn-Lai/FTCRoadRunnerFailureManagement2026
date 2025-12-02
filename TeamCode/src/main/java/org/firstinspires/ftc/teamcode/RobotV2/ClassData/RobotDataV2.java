package org.firstinspires.ftc.teamcode.RobotV2.ClassData;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.security.spec.EllipticCurve;
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
        turret = new Turret(hardwareMap);
        carosel = new Carosel(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap);
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
    public boolean isBusy(DcMotorEx motor){
        //Defaults to 0 if no target pos
        return Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) > RobotConstantsV2.MOTOR_TOLERENCE;
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

        public DriveTrain(HardwareMap hardwareMap){

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

        /** Gamepad */
        public void setGamepad(Gamepad driver) {
            this.driver = driver;
        }

        /** Main Modes */
        public void switchMode(){
            if (currentMode.equals(RobotConstantsV2.mainModes[0])){
                currentMode = RobotConstantsV2.mainModes[1];
                carosel.wipeInventory();
            }
            else currentMode = RobotConstantsV2.mainModes[0]; //Default to Auto Mode (Even if humanIntake)
        }
        public void switchHumanIntake(){

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
        public void requestSortedFire(){ //TODO && LimeLightVision.isFoundMotif
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

        /** Movement */
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

            if (currentRuntime >= 90 && currentRuntime <= 92){
                driver.rumble(2);
                telemetry.addLine("END GAME!");
            }

            else if (currentRuntime >= 110 && currentRuntime <= 120){
                driver.rumble(2);
                double time = 120 - currentRuntime;
                telemetry.addLine("END GAME!");
                telemetry.addLine(Math.floor(time) + " Seconds Left!");
            }
        }
        public void telemetryDriveTrain(){
            telemetry.addData("Time: ", Math.floor(getRuntime()) + " Seconds");
            telemetry.addData("Current Mode: ", currentMode);
            telemetry.addData("Current Sub Mode: ", currentSubMode);
        }


        public DcMotor getLFmotor(){
            return LFmotor;
        } //TODO Temporarily To Test Auto Park

    }

    //----------------------------------------

    public class PIDControl{

        /** Constants */
        private double kP = 0.71;
        private double kI = 0.0005;
        private double kD = 0.35;
        private double kF = 1;

        /** Utility */
        private ElapsedTime PIDTimer = new ElapsedTime();
        private double integralSum = 0;
        private double lastError = 0;

        //----------------------------------------

        /** Constants */
        public String getPIDCos(){
            return String.format("kP: %f, kI: %f, kD: %f, kF: %f", kP, kI, kD, kF);
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
        } //TODO Tune this
    } //TODO Update Tester File

    //----------------------------------------

    public class Turret extends PIDControl{

        /** Parts */
        public DcMotorEx shooterMotor;
        private Servo shooterServo;

        /** Physics Parameters */
        private final double a = -9.8; //Acceleration m/s^2
        public double heightOfLauncher = 0.3556; //TODO Update this
        private final double height = 1.2 - heightOfLauncher; //Meters 1.05
        private final double vY = Math.sqrt(2 * -a * height);
        private double vX = 0;
        private double vX0 = 0; //initial vX of the robot at an instant in time

        /** Fly Wheel */
        private final double spinWheelRadius = 0.096/2; //Meters
        private final double w = 28.0; //Ticks

        /** Toggle Turret */
        private boolean toggleTurretAim;
        private boolean isFar;

        //----------------------------------------

        /** Constructor */
        private Turret(HardwareMap hardwareMap){

            /** Hardware Init */
            shooterMotor = (DcMotorEx)hardwareMap.get(DcMotor.class,"shooterMotor");
            shooterServo = hardwareMap.get(Servo.class,"shooterServo");

            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
            return Math.sqrt(Math.pow(getVy(),2) + Math.pow(getVx(disp),2)); //TODO Removed Velocity Drag Multiplier and moved to ticks
        }
        private double getAngleTotal(double disp){
            return Math.atan2(getVy(),getVx(disp)) * (180 / Math.PI);
        }
        private double getSpinWheelRadius(){
            return spinWheelRadius;
        }
        private double getRadiansPerSecond(double disp){ //From Velocity
            return getVelocityTotal(disp)/getSpinWheelRadius();
        }
        private double getDesiredRPM(double disp){ //From Radians Per SEcond
            return getRadiansPerSecond(disp) * 60 / (2 * Math.PI);
        }
        private double getW(){
            return w;
        }
        private double getTPS(double disp){ //From Radians per Second

            double TPSfound = ((getRadiansPerSecond(disp) * getW()) / (2 * Math.PI)) * RobotConstantsV2.dragMultiplier;

            if (TPSfound > 1800){
                return 1800;
            }

            return TPSfound;
        }

        /** Toggle Turret Modes */
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
        public boolean isFarToggled(){
            return isFar;
        }

        /** Human Player Intake Mode */
        public void activateHumanIntakeMode(){
            shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
            powerShooterMotor(RobotConstantsV2.HUMAN_INTAKE_SPEED);
        }
        public void deactivateHumanIntakeMode(){
            shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        }

        /** Fly Wheel Power */
        public void powerShooterMotor(double TPS){
            double current = shooterMotor.getVelocity();
            shooterMotor.setVelocity(PIDShooter(current,TPS));
        }
        public void killShooter(){
            shooterMotor.setVelocity(RobotConstantsV2.KILL_SHOOTER_SPEED);
        }

        /** Hood */
        private double convertDegToServo(double angle){

            if (angle > RobotConstantsV2.MAX_HOOD_ANGLE) return 0.2;
            else if (angle < RobotConstantsV2.MIN_HOOD_ANGLE) return 0.9;

            return (double) Math.round((-0.0388889 * angle + 2.72778) * 1000) / 1000;
        }
        public void angleRobot(double disp){
            double desiredAngle = getAngleTotal(disp) + RobotConstantsV2.ANGLE_BONUS;
            shooterServo.setPosition(convertDegToServo(desiredAngle));
            telemetry.addData("a",convertDegToServo(desiredAngle));
            telemetry.addData("x",desiredAngle);
        }

        /** Final Shot */
        public void aimBall(double disp){
            angleRobot(disp);
            double TPS = getTPS(disp);
            powerShooterMotor(TPS);
        }

        public void aimBallManual(boolean isFarNow, double disp){

            if (isFarNow){
                angleRobot(disp);
                powerShooterMotor(RobotConstantsV2.FAR_TPS);
            }

            else{
                angleRobot(disp);
                double TPS = getTPS(disp);
                powerShooterMotor(TPS);
            }
        }

        /** Final Updates */
        public void telemetryTurretBasic(){
            telemetry.addData("Turret On: ", toggleTurretAim);
            telemetry.addData("Close Shot: ", !isFar);
        }
        public double getTPSError(double disp){
            return shooterMotor.getVelocity() - getTPS(disp);
        }

        public double getFarShotTPSError(){
            return shooterMotor.getVelocity() - RobotConstantsV2.FAR_TPS;
        }
        private double getManualTPSError(double TPS){
            return shooterMotor.getVelocity() - TPS;
        }
        public void telemetryTurret(double disp){
            telemetry.addLine("Kinematics: \n");

            telemetry.addData("Total Velocity: ", getVelocityTotal(disp));
            telemetry.addData("vY: ", getVy());
            telemetry.addData("vX: ",getVx(disp));
            telemetry.addData("vX0: ", getInitalVelocityX());
            telemetry.addData("Estimated Drag Multiplier: ", RobotConstantsV2.dragMultiplier);
            telemetry.addLine("-------------------------- \n");

            telemetry.addLine("TPS: \n");

            telemetry.addData("True TPS: ", shooterMotor.getVelocity());
            telemetry.addData("Ticks Per Second: ", getTPS(disp));
            telemetry.addData("TPS Error:", getTPSError(disp));

            telemetry.addLine("-------------------------- \n");

            telemetry.addLine("Outputs: \n");

            telemetry.addData("Time Extrema:", getTimeExtrema());
            telemetry.addData("Displacement (m): ", disp);
            telemetry.addData("Angle: " , getAngleTotal(disp));
            telemetry.addData("True Servo Angle:",convertDegToServo(getAngleTotal(disp)));

        }
    }

    //----------------------------------------

    public class Carosel{

        /** Hardware */
        private DcMotorEx intakeMotor;
        private DcMotorEx caroselMotor;
        private Servo transferServo;


        /** Sensors */
        private Servo indicatorOne;
        private Servo indicatorTwo;
        private ColorSensor colorSensor;
        private DistanceSensor colorSensorDist;
        private DigitalChannel transferBeam;

        /** Storage */
        private String[] inventory;
        private String[] pattern;
        private int currentCycle;

        /** Carosel Booleans */
        private boolean intakeMotorOn;
        private boolean transferUp;
        private boolean autoIntakeCooldownActive;

        /** Transfer & Shoot */

        private ElapsedTime autoIntakeCoolDown;

        private boolean cycleInProg;
        private boolean patternInProg;
        private String currentSubModeQueue;
        private boolean transferCooldownActive;
        private ElapsedTime transferCooldown;
        private boolean transferInProg;
        private int rapidFireCurrentShotCount;
        private int sortedFireCurrentShotCount;
        private boolean shotSucceed;
        private boolean failsafeSubmode;
        private ElapsedTime failsafeTimer;
        private int maxShots;

        //----------------------------------------

        /** Constructor */
        public Carosel(HardwareMap hardwareMap){

            /** Hardware Init */

            intakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class,"intakeMotor");
            caroselMotor = (DcMotorEx) hardwareMap.get(DcMotor.class,"caroselMotor");
            transferServo = hardwareMap.get(Servo.class,"transferServo");
            indicatorOne = hardwareMap.get(Servo.class,"indicatorOne");
            indicatorTwo = hardwareMap.get(Servo.class,"indicatorTwo");
            colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");
            colorSensorDist = (DistanceSensor) colorSensor;
            transferBeam = hardwareMap.get(DigitalChannel.class,"transferBeam");

            intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
            caroselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            caroselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //TODO dont zero this when we run auto

            /** Spindex */
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

            failsafeTimer = new ElapsedTime();
            transferCooldown = new ElapsedTime();
            autoIntakeCoolDown = new ElapsedTime();

            rapidFireCurrentShotCount = 0;
            sortedFireCurrentShotCount = 0;
            //sortedFireQueue = 0;
        }

        //----------------------------------------

        /** Autonomous */
        public void setInventoryAuto(){
            inventory = new String[]{"Green","Purple","Purple"};
        }

        /** Intake */
        public void switchIntake(){
            if (!intakeMotorOn){
                intakeMotor.setVelocity(RobotConstantsV2.INTAKE_ON);
                intakeMotorOn = true;
            }
            else{
                intakeMotor.setVelocity(RobotConstantsV2.INTAKE_OFF);
                intakeMotorOn = false;
            }
        }
        public void forceIntakeOn(){
            intakeMotor.setVelocity(RobotConstantsV2.INTAKE_ON);
            intakeMotorOn = true;
        }
        public void forceIntakeOff(){
            intakeMotor.setPower(RobotConstantsV2.INTAKE_OFF);
            intakeMotorOn = false;
        }

        /** Pattern */
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
            if (inventory[currentCycle].equals("Green") || inventory[currentCycle].equals("Purple") || !isCaroselInPlace()){
                return;
            }
            telemetry.addData("Current Cycle NOW: ", currentCycle);
            telemetry.addData("Current Inv Spoot NOW: ", inventory[currentCycle]);
            inventory[currentCycle] = getIntakeColor();
        }
        private void wipeInventory(){
            inventory = new String[]{"Empty","Empty","Empty"};
        }

        /** Transfer */
        //TODO Make this more clean

        public void toggleTransfer(){
            if (!transferUp){
                inventory[currentCycle] = "Empty";
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

        public boolean isTransferCooldownActive(){
            return transferCooldownActive;
        }
        public void transferStartTimer(){
            if (turret.isToggleTurretAim() && !driveTrain.getCurrentMode().equals(RobotConstantsV2.mainModes[2])){
                transferCooldownActive = true;
                forceTransferUp();
                transferCooldown.reset();
            }
        }
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
            if (transferInProg){ //
                transferInProg = false;
                transferCooldownActive = true;
                forceTransferUp();
                transferCooldown.reset();
                startFailsafeSubmode();
            }
        }

        public void endFailsafeSubmode(){
            failsafeSubmode = false;
        }

        public void startFailsafeSubmode(){
            failsafeSubmode = true;
            failsafeTimer.reset();
        }
        public boolean isFailsafeSubmode(){
            if (!shotSucceed && failsafeSubmode && failsafeTimer.milliseconds() > RobotConstantsV2.FAILSAFE_SUBMODE_TIMER){
                failsafeSubmode = false;
                return true;
            }
            else return false;
        }

        public boolean getTransferInProg(){
            return transferInProg;
        }

        public void checkShotSuccess(){
            if (isShotSuccess()){
                shotSucceed = true;
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
        //TODO Currently Cycling has wrapping, might need to implement hold system -_- for shortest path

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
        public void cycleRapidFire(){
            //Only Activates Once
            //TODO incrementRapidFireCurrentShotCount(); TRANSFER
            if (cycleInProg){
                if (!detectedArtifact()){
                    cycleCaroselManual();
                }
                cycleInProg = false;
            }
        }
        public void cycleSortedFire(int patternNum){
            if (patternInProg && patternNum <= 2) {
                cyclePattern(patternNum);
                patternInProg = false;
            }
        }

        public boolean isReadyShoot(){
            return !inventory[currentCycle].equals("Empty");
        }
        private String getTransferColor(){ //TODO REDO WITH NEW 4th POS
            return inventory[currentCycle];
        }
        public boolean isEmptySpot(){
            return Arrays.asList(inventory).contains("Empty");
            //return !(getEmptySpot() == -1);
        }
        public boolean isShotSuccess(){
            return transferBeam.getState();
        }
        public void cycleCaroselManual(){
            cycleCarosel(getNextCycle()); //TODO Previous cycle too
        }
        public int getEmptySpot(){

//            int index = 0;
//            for (String i: inventory){
//                if (i.equals("Empty")){
//                    return index;
//                }
//                index ++;
//            }

            return Arrays.asList(inventory).indexOf("Empty");
            //return -1;
        }

        /** Intake Cycling */
        public void autoIntakeCycle(){
            telemetry.addData("Next Empty Spot: ", getEmptySpot());
            if (!transferCooldownActive && detectedArtifact() && isCaroselInPlace() && isEmptySpot()){
                updateInventory();
                autoIntakeCooldownActive = true;
            }
        }

        public void receiveAutoCycleStatus(){
            if (autoIntakeCooldownActive && autoIntakeCoolDown.milliseconds() > RobotConstantsV2.COOLDOWN_INTAKE){
                autoIntakeCooldownActive = false;
                if (isEmptySpot()) cycleCarosel(getEmptySpot());
            }
        }

        /** Spindex */
        public void cycleCarosel(int desiredCycle){
            if (!transferCooldownActive){
                currentCycle = desiredCycle;

                caroselMotor.setTargetPosition(RobotConstantsV2.caroselPos[desiredCycle]);
                caroselMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                caroselMotor.setVelocity(RobotConstantsV2.CAROSEL_SPEED);
            }
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
        public void cyclePattern(int number){
            int slot = getInvPosition(pattern[number]);
            //int trashArtifact = getNearestUselessArtifact();

            if (slot == -1){
                //if (trashArtifact != -1) cycleCarosel(trashArtifact);
                if (maxShots > sortedFireCurrentShotCount) cycleCarosel(getNearestArtifact());
                return;
            }

            cycleCarosel(slot);
        }
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
        public int getCurrentCycle(){
            return currentCycle;
        }

        /** Intake Color Detection */
        public float[] getHSV(){

            float[] hsv = new float[]{0F,0F,0F};
            Color.RGBToHSV(colorSensor.red()*255,colorSensor.green()*255,colorSensor.blue()*255, hsv);
            return hsv;
        }
        public boolean detectedArtifact(){
            return colorSensorDist.getDistance(DistanceUnit.CM) < 3 || !inventory[currentCycle].equals("Empty");
        } //TODO CHECK OVER
        public String getIntakeColor() { //TODO make these static variables for tuning
            if (detectedArtifact()) {
                float[] HSV = getHSV();

                if (HSV[0] < 200 && HSV[1] >= 0.55){
                    return "Green";
                }
                else if (HSV[0] >= 200 && HSV[1] < 0.55){
                    return "Purple";
                }
            }

            return "Empty";
        }
        public void indicatorsInInit(){
            indicatorOne.setPosition(RobotConstantsV2.INDICATOR_ORANGE);
            indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_ORANGE);
        }

        /** Indicator Lights */
        public void updateIndicators(String mode, double disp, LimeLightVision limelight){

            switch (mode){
                case("auto"):

                    if (!limelight.getResults().isValid()){
                        disp = RobotConstantsV2.CLOSE_BALL_DISTANCE;
                    }

                    //Motor Speed (Don't Need Multiple Colors)
                    telemetry.addData("Stupid TPS Error: ", turret.getTPSError(disp));
                    telemetry.addData("Attempted TPS: ", turret.getTPS(disp));
                    telemetry.addData("Threshold Max: ", turret.getTPS(disp) * RobotConstantsV2.SHOOTER_MAX_SPEED_THRESHOLD);

                    //TODO maybe another color for when no vision but can shoot
                    if (!limelight.getResults().isValid() || !turret.isToggleTurretAim() || turret.getTPSError(disp) > turret.getTPS(disp) * RobotConstantsV2.SHOOTER_MAX_SPEED_THRESHOLD || turret.getTPSError(disp) < turret.getTPS(disp) * RobotConstantsV2.SHOOTER_MIN_SPEED_THRESHOLD){
                        indicatorOne.setPosition(RobotConstantsV2.INDICATOR_RED);
                    }
                    else{
                        indicatorOne.setPosition(RobotConstantsV2.INDICATOR_BLUE);
                    }

                    //Current Color

                    //Live View (also invententory for redundency
                    if (getIntakeColor().equals("Purple")){
                        indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_PURPLE);
                    }
                    else if (getIntakeColor().equals("Green")){
                        indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_GREEN);
                    }
                    else{
                        if (inventory[currentCycle].equals("Purple")){
                            indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_PURPLE);
                        }
                        else if (inventory[currentCycle].equals("Green")){
                            indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_GREEN);
                        }
                        else{
                            indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_YELLOW);
                        }
                    }

                    break;

                case("manual"):

                    if (turret.isFarToggled()){
                        disp = RobotConstantsV2.FAR_BALL_DISTANCE;

                        if (turret.getFarShotTPSError() > RobotConstantsV2.FAR_TPS * RobotConstantsV2.SHOOTER_MAX_SPEED_THRESHOLD || turret.getFarShotTPSError() < RobotConstantsV2.FAR_TPS * RobotConstantsV2.SHOOTER_MIN_SPEED_THRESHOLD){
                            indicatorOne.setPosition(RobotConstantsV2.INDICATOR_RED);
                        }
                        else{
                            indicatorOne.setPosition(RobotConstantsV2.INDICATOR_BLUE);
                        }
                    }

                    else{
                        disp = RobotConstantsV2.CLOSE_BALL_DISTANCE;
                        //WOrks because using TPS here
                        if (turret.getTPSError(disp) > RobotConstantsV2.FAR_TPS * RobotConstantsV2.SHOOTER_MAX_SPEED_THRESHOLD || turret.getTPSError(disp) < RobotConstantsV2.FAR_TPS * RobotConstantsV2.SHOOTER_MIN_SPEED_THRESHOLD){
                            indicatorOne.setPosition(RobotConstantsV2.INDICATOR_RED);
                        }
                        else{
                            indicatorOne.setPosition(RobotConstantsV2.INDICATOR_BLUE);
                        }
                    }


                    //Current Color
                    //Live View (also invententory for redundency
                    if (getIntakeColor().equals("Purple")){
                        indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_PURPLE);
                    }
                    else if (getIntakeColor().equals("Green")){
                        indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_GREEN);
                    }
                    else{
                        indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_YELLOW);
                    }

                    break;

                case("humanIntake"):

                    indicatorOne.setPosition(RobotConstantsV2.INDICATOR_RED); //Shows that Human Player Mode

                    //Double Check Make Sure it empty currently
                    if (!isEmptySpot()){
                        //Inventory is Full
                        indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_RED);
                    }
                    else if (!detectedArtifact() || inventory[currentCycle].equals("Empty") || isCaroselInPlace()){
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

        /** Magnetic Limit Switch */
        public boolean isCaroselInPlace(){
            return !isBusy(caroselMotor);
        }

        /** Final Updates */
        public void telemetryCarosel(){
            telemetry.addLine(String.format("Inventory: (%s, %s, %s)", inventory[0], inventory[1], inventory[2]));
            telemetry.addLine(String.format("Motif: (%s, %s, %s)", pattern[0], pattern[1], pattern[2]));
            float[] HSV = getHSV();
            telemetry.addLine(String.format("HSV: (%.5f, %.5f, %.5f)", HSV[0], HSV[1], HSV[2]));
            telemetry.addData("Current Cycle: ", currentCycle);
            telemetry.addData("Identified Color: ", getIntakeColor());
            telemetry.addData("Inventory Current: ", inventory[currentCycle]);
            telemetry.addData("Has Pattern in Inventory: ", hasPattern());
        }
    }
}