package org.firstinspires.ftc.teamcode.RobotV2.ClassData;

import android.graphics.Color;
import android.icu.number.NumberRangeFormatter;

import androidx.core.graphics.drawable.RoundedBitmapDrawable;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
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
        } //TODO clean this up
    }

    //----------------------------------------

    public class PIDControl{

        /** PID For Shooter*/
        private double kP = 0.006;
        private double kI = 0.0000000001;
        private double kD = 0.00000012;
        private double kF = 0.0004;
        private ElapsedTime PIDTimer = new ElapsedTime();
        private double integralSum = 0;
        private double lastError = 0;

        /** PID for Carosel */
        private double kPC = 0.005;
        private double kIC = 0;
        private double kDC = 0;
        private ElapsedTime PIDTimerC = new ElapsedTime();
        private double integralSumC = 0;
        private double lastErrorC = 0;

        //----------------------------------------

        /** Get Constants for Shooter */
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
        }

        public double PIDCarosel(double current, double desired) {

            double currentTimeC = PIDTimer.seconds();

            double outputC = 0;

            double errorC = desired - current;

            integralSumC += errorC * currentTimeC;

            double derivateC = (errorC - lastErrorC) / currentTimeC;

            lastError = errorC;

            PIDTimerC.reset();

            outputC = (errorC * kPC) + (derivateC * kDC) + (integralSumC * kIC);

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
        private final double height = 1.2 - heightOfLauncher;
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
            telemetry.addData("TPS NOW", TPS);
            telemetry.addData("TPS Current: ", current);

        }
        public void killShooter(){
            //shooterMotor.setVelocity(RobotConstantsV2.KILL_SHOOTER_SPEED);
            shooterMotor.setPower(0);
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
        public void aimBall(double disp, double TPS){
                angleRobot(disp);
                powerShooterMotor(TPS);
        }

        /** Final Updates */
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
        public void telemetryTurretBasic(){
            telemetry.addData("Turret On: ", toggleTurretAim);
            telemetry.addData("Close Shot: ", !isFar);
        }
        public double getTPSError(double disp){
            return shooterMotor.getVelocity() - getTPS(disp);
        }
        public double getTPSErrorManual(double TPS){
            return shooterMotor.getVelocity() - TPS;
        }
    }

    //----------------------------------------

    public class Carosel extends PIDControl{

        /** Hardware */
        private DcMotorEx intakeMotor;
        private Servo transferServo;
        private CRServo caroselServo;
        private AnalogInput caroselAnalog;


        /** Sensors */
        private Servo indicatorOne;
        private Servo indicatorTwo;
        private ColorSensor colorSensorFront;
        private ColorSensor colorSensorBack;
        private AnalogInput rangerSensor; //TODO might have to use binary if possible
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
        private ElapsedTime cycleTransferCooldown; //Can't cycle while transfer acitve
        private boolean cycleTransferCooldownActive; //TODO honestly forgot what this does

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
            caroselAnalog = hardwareMap.get(AnalogInput.class,"analogCarosel");
            transferServo = hardwareMap.get(Servo.class,"transferServo");

            /** Sensors Init */
            indicatorOne = hardwareMap.get(Servo.class,"indicatorOne");
            indicatorTwo = hardwareMap.get(Servo.class,"indicatorTwo");
            colorSensorFront = hardwareMap.get(ColorSensor.class,"colorSensorFront");
            colorSensorBack  = hardwareMap.get(ColorSensor.class,"colorSensorBack");
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

            cycleTransferCooldownActive = false;
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

            /** Timers */
            failsafeTimer = new ElapsedTime();
            transferCooldown = new ElapsedTime();
            autoIntakeCoolDown = new ElapsedTime();
            cycleTransferCooldown = new ElapsedTime();

            /** Sub Modes */
            rapidFireCurrentShotCount = 0;
            sortedFireCurrentShotCount = 0;
        }

        //----------------------------------------

        public void resetBooleans(){
            cycleTransferCooldownActive = false;
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


        /** Intake */
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
            intakeMotor.setVelocity(RobotConstantsV2.INTAKE_ON);
            intakeMotorOn = true;
        }
        public void forceIntakeOff(){
            intakeMotor.setPower(RobotConstantsV2.INTAKE_OFF);
            intakeMotorOn = false;
        }
        public void checkForAutoEject(){
            if (!isEmptySpot() && !ejectActive){
                intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
                intakeMotor.setVelocity(RobotConstantsV2.INTAKE_IN_EJECT);
                intakeMotorOn = true;
                ejectActive = true;
            }

            else if (isEmptySpot()){
                ejectActive = false;
            }
        }


        /** Transfer */
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
            transferCooldownActive = false;
        }
        public void transferStartTimer(){
            if (turret.isToggleTurretAim() && !driveTrain.getCurrentMode().equals(RobotConstantsV2.mainModes[2])){
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
            if (transferInProg){ //
                transferInProg = false;
                transferCooldownActive = true;
                forceTransferUp();
                transferCooldown.reset();
                startFailsafeSubmode();
            }
        }
        public void startTransferCooldown(){
            if (!cycleTransferCooldownActive){
                cycleTransferCooldown.reset();
                cycleTransferCooldownActive = true;
            }
        }
        public void resetTransferStat(){
            cycleTransferCooldownActive = false;
        }
        public void cycleTransferStartTimer(){
            if (transferInProg && cycleTransferCooldownActive && cycleTransferCooldown.milliseconds() > RobotConstantsV2.TRANSFER_COOLDOWN){ //

                transferInProg = false;
                transferCooldownActive = true;
                forceTransferUp();
                transferCooldown.reset();
                startFailsafeSubmode();
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
        public void cycleCaroselManual(){
            cycleCarosel(getNextCycle());
        }
        public void autoIntakeCycle(){
            //telemetry.addData("Next Empty Spot: ", getEmptySpot());
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
            if (autoIntakeCooldownActive && autoIntakeCoolDown.milliseconds() > RobotConstantsV2.COOLDOWN_INTAKE){
                autoIntakeCooldownActive = false;
                if (isEmptySpot()) cycleCarosel(getEmptySpot());
            }
        }
        public void cycleCarosel(int desiredCycle){
            if (!transferCooldownActive){
                int increm = getCycleIncrement(desiredCycle) + RobotConstantsV2.CAROSEL_TOUCHPAD;
                currentCycle = desiredCycle;
                runCaroselToPos(increm);
            }
        }
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


        /** Carosel Servo Logic */
        public double getCaroselDegrees(){
            return caroselAnalog.getVoltage() * RobotConstantsV2.encoderRes;
        }
        public boolean isBusy(){
            double targetPos = getCycleIncrement(currentCycle) + RobotConstantsV2.CAROSEL_TOUCHPAD; //TODO test this
            return Math.abs(targetPos - getCaroselDegrees()) > RobotConstantsV2.CAROSEL_TOLERANCE;
        }
        public boolean isCaroselInPlace(){
            return !isBusy();
        }

        private void runCaroselToPos(double pos){
            caroselServo.setPower(-PIDCarosel(globalCaroselPosition,pos));
        }
        private double getCaroselIncrement(double diff, double current){

            double step = 0;

            double N = 3.3;

            double diffe = diff - current;

            double modDiff = ((diffe % N) + N) % N;

            if (modDiff > N/ 2){
                step = (modDiff - N);
            }
            else{
                step = (modDiff);
            }

            return step * RobotConstantsV2.encoderRes;
        } //this is used for tracking global encoder pos
        private void updateCaroselEncoder(){
            double current = getCaroselDegrees();
            double in = getCaroselIncrement(current,lastCaroselPosition);
            lastCaroselPosition = current;
            globalCaroselPosition += in;
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
                step = (modDiff - N) * RobotConstantsV2.CAROSEL_INCREMENT;
            }
            else{
                step = (modDiff) * RobotConstantsV2.CAROSEL_INCREMENT;
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
        public float[] getHSV(){

            float[] hsv = new float[]{0F,0F,0F};
            Color.RGBToHSV(colorSensorFront.red()*255, colorSensorFront.green()*255, colorSensorFront.blue()*255, hsv);
            return hsv;
        }
        public double getRangerDistance(){
            return (rangerSensor.getVoltage() * RobotConstantsV2.RANGER_EQU_SLOPE + RobotConstantsV2.RANGER_EQU_Y_INT) * RobotConstantsV2.IN_TO_CM;
        }
        public boolean detectedArtifact(){
            return (getRangerDistance() > RobotConstantsV2.RANGER_DETECTION_MIN_THRESHOLD && getRangerDistance() < RobotConstantsV2.RANGER_DETECTION_MAX_THRESHOLD) || !inventory[currentCycle].equals("Empty");
        }
        public String getIntakeColor() { //TODO make these static variables for tuning
            if (detectedArtifact()) {
                float[] HSV = getHSV();

                if (HSV[0] < RobotConstantsV2.MIDDLE_H && HSV[1] >= RobotConstantsV2.MIDDLE_S){
                    return "Green";
                }
                else if (HSV[0] >= RobotConstantsV2.MIDDLE_H && HSV[1] < RobotConstantsV2.MIDDLE_S){
                    return "Purple";
                }
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

                    if (!limelight.getResults().isValid()){
                        disp = RobotConstantsV2.CLOSE_BALL_DISTANCE;
                    }

                    //Motor Speed (Don't Need Multiple Colors)
                    telemetry.addData("Stupid TPS Error: ", turret.getTPSError(disp));
                    telemetry.addData("Attempted TPS: ", turret.getTPS(disp));
                    telemetry.addData("Threshold Max: ", turret.getTPS(disp) * RobotConstantsV2.SHOOTER_SPEED_THRESHOLD);

                    //TODO maybe another color for when no vision but can shoot
                    if (!limelight.getResults().isValid() || !turret.isToggleTurretAim() || Math.abs(turret.getTPSError(disp)) > turret.getTPS(disp) * RobotConstantsV2.SHOOTER_SPEED_THRESHOLD){
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
                        //disp = RobotConstantsV2.FAR_BALL_DISTANCE;

//                        if (turret.getFarShotTPSError() > RobotConstantsV2.FAR_TPS * RobotConstantsV2.SHOOTER_MAX_SPEED_THRESHOLD || turret.getFarShotTPSError() < RobotConstantsV2.FAR_TPS * RobotConstantsV2.SHOOTER_MIN_SPEED_THRESHOLD){
//                            indicatorOne.setPosition(RobotConstantsV2.INDICATOR_RED);
//                        }

                        telemetry.addData("Distance: ", disp);
                        telemetry.addData("Stupid TPS Error: ", turret.getTPSError(disp));
                        telemetry.addData("Attempted TPS: ", turret.getTPS(disp));
                        telemetry.addData("Threshold Max: ", turret.getTPS(disp) * RobotConstantsV2.SHOOTER_SPEED_THRESHOLD);

                        if (Math.abs(turret.getTPSError(disp)) > turret.getTPS(disp) * RobotConstantsV2.SHOOTER_SPEED_THRESHOLD){
                            indicatorOne.setPosition(RobotConstantsV2.INDICATOR_RED);
                        }

                        else{
                            indicatorOne.setPosition(RobotConstantsV2.INDICATOR_BLUE);
                        }
                    }

                    else{
                        disp = RobotConstantsV2.CLOSE_BALL_DISTANCE;
                        //WOrks because using TPS here
                        if (Math.abs(turret.getTPSError(disp)) > turret.getTPS(disp) * RobotConstantsV2.SHOOTER_SPEED_THRESHOLD){
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

                    indicatorOne.setPosition(RobotConstantsV2.INDICATOR_ORANGE); //Shows that Human Player Mode

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