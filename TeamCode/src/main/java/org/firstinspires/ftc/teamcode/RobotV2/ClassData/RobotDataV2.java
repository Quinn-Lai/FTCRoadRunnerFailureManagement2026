package org.firstinspires.ftc.teamcode.RobotV2.ClassData;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.opencv.ColorRange;

import java.util.Arrays;

public class RobotDataV2 {

    /** Components */
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private static ElapsedTime runtime;

    /** Auto Status */
    private boolean pendingColor;
    private boolean pendingPosition;

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

        turret = new Turret(hardwareMap);
        carosel = new Carosel(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap);
        runtime = null;
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
        return Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) > RobotConstantsV2.MOTOR_TOLERENCE;
    }

    /** Auto Status */
    public boolean isPendingColor(){
        return pendingColor;
    }
    public boolean isPendingPosition(){
        return pendingPosition;
    }
    public void selectedColor(){
        pendingColor = false;
    }
    public void selectedPosition(){
        pendingPosition = false;
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
            pendingColor = true;
            pendingPosition = true;

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
            if (currentMode.equals(RobotConstantsV2.mainModes[0])) currentMode = RobotConstantsV2.mainModes[1];
            else currentMode = RobotConstantsV2.mainModes[0]; //Default to Auto Mode (Even if humanIntake)
        }
        public void switchHumanIntake(){
            currentMode = RobotConstantsV2.mainModes[2];
        }
        public String getCurrentMode(){
            return currentMode;
        }

        /** Sub Modes */
        public String getCurrentSubMode(){
            return currentSubMode;
        }
        public void requestRapidFire(){
            currentSubMode = RobotConstantsV2.subModes[0];
        }
        public void requestSortedFire(){
            currentSubMode = RobotConstantsV2.subModes[1];
        }
        public void endSubMode(){
            currentMode = RobotConstantsV2.subModes[2];
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
        public void updateModeColor(Turret turret){
            if (currentMode.equals(RobotConstantsV2.mainModes[0])){
                driver.setLedColor( 0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
            }
            else if (currentMode.equals(RobotConstantsV2.mainModes[1])){
                //Differentiate these two
                if (turret.isFarToggled()){
                    driver.setLedColor( 0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
                }
                else{
                    driver.setLedColor( 0, 0.5, 1, Gamepad.LED_DURATION_CONTINUOUS);
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
        private double kP = 6.7;
        private double kI = 0.53;
        private double kD = 0.3;
        private double kF = 0;

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
            return ((getRadiansPerSecond(disp) * getW()) / (2 * Math.PI)) * RobotConstantsV2.dragMultiplier;
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
        public void activateHumanIntakeMode(Carosel carosel){
            shooterMotor.setDirection(DcMotor.Direction.REVERSE);
            powerShooterMotor(RobotConstantsV2.HUMAN_INTAKE_SPEED);

            carosel.autoIntakeCycle();

        }
        public void deactivateHumanIntakeMode(){
            shooterMotor.setDirection(DcMotor.Direction.FORWARD);
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
            return -0.031506 * angle + 1.99244;
        }
        public void angleRobot(double disp){
            double desiredAngle = getAngleTotal(disp) + RobotConstantsV2.ANGLE_BONUS;
            shooterServo.setPosition(convertDegToServo(desiredAngle));
        }

        /** Final Shot */
        public void aimBall(double disp){
            angleRobot(disp);
            double TPS = getTPS(disp);

            if (disp < 2.5){
                powerShooterMotor(TPS);
                //shooterMotor.setVelocity(TPS);
            }
            else{
                //powerShooterMotor(RobotConstantsV2.FAR_TPS); //TODO Separted On Purpose
                powerShooterMotor(TPS); //TODO Test if can use far default or if camera can pick up
                //shooterMotor.setVelocity(RobotConstantsV2.longTPS);
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
        private DigitalChannel magLS;
        private ColorSensor colorSensor;
        private DistanceSensor colorSensorDist;
        private DistanceSensor transferBeam;
        private DistanceSensor intakeBeam;

        /** Storage */
        private String[] inventory;
        private String[] pattern;
        private int currentCycle;

        /** Carosel Booleans */
        private boolean intakeMotorOn;
        private boolean transferUp;

        /** Transfer & Shoot */
        private int sortedFireQueue;
        private boolean shotInProg;
        private boolean transferCooldownActive;
        private ElapsedTime transferCooldown;
        private int availableShots;

        //----------------------------------------

        /** Constructor */
        public Carosel(HardwareMap hardwareMap){

            /** Hardware Init */

            intakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class,"intakeMotor");
            caroselMotor = (DcMotorEx) hardwareMap.get(DcMotor.class,"caroselMotor");
            transferServo = hardwareMap.get(Servo.class,"transferServo");
            indicatorOne = hardwareMap.get(Servo.class,"indicatorOne");
            indicatorTwo = hardwareMap.get(Servo.class,"indicatorTwo");
            magLS = hardwareMap.get(DigitalChannel.class,"magLS");
            colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");
            colorSensorDist = hardwareMap.get(DistanceSensor.class,"colorSensor");
            transferBeam = hardwareMap.get(DistanceSensor.class,"transferBeam");
            intakeBeam = hardwareMap.get(DistanceSensor.class,"intakeBeam");

            /** Spindex */
            inventory = new String[]{"Empty","Empty","Empty"};
            pattern = new String[]{"Empty","Empty","Empty"};

            /** Variable Init */

            intakeMotorOn = false;
            transferUp = false;
            shotInProg = false;
            transferCooldownActive = false;

            transferCooldown = new ElapsedTime();

            availableShots = 0;
            sortedFireQueue = 0;
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
                intakeMotor.setPower(RobotConstantsV2.INTAKE_OFF);
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
        private boolean hasPattern() {

            String[] tempArrayInventory = inventory;
            String[] tempArrayPattern = pattern;

            Arrays.sort(tempArrayInventory);
            Arrays.sort(tempArrayPattern);

            return Arrays.equals(tempArrayInventory,tempArrayPattern);
        }
        private void updateInventory(){
            //Won't override updates
            if (inventory[currentCycle].equals("Green") || inventory[currentCycle].equals("Purple")){
                return;
            }
            inventory[currentCycle] = getIntakeColor();
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
        private void checkForShotOpportunity(){
            if (isCaroselInPlace() && !shotInProg && isReadyShoot()){ //TODO There is nothing to prevent an accdiental mag ls read from killing everything
                forceTransferUp();
                shotInProg = true;
            }
        }
        private void checkResetTransfer(){
            if (shotInProg && isShotSuccess()){
                forceTransferDown();
                shotInProg = false;
                transferCooldownActive = true;
                transferCooldown.reset();
            }
        }
        private void checkEndShotCoolDown(){
            if (!shotInProg && transferCooldownActive && transferCooldown.milliseconds() > RobotConstantsV2.COOLDOWN_SHOT){
                transferCooldownActive = false;
            }
        }
        private void updateAvailableShots(){

            int shotCounter = 0;

            for (String i:inventory){
                if (i.equals("Green") || i.equals("Purple")){
                    shotCounter ++;
                }
            }

            availableShots = shotCounter;
        }
        public boolean isReadyShoot(){
            return !inventory[currentCycle].equals("Empty");
        }
        private String getTransferColor(){ //TODO REDO WITH NEW 4th POS
            return inventory[currentCycle];
        }
        private boolean isEmptySpot(){
            for (String i: inventory){
                if (i.equals("Empty")){
                    return true;
                }
            }
            return false;
        }
        public boolean isShotSuccess(){
            return transferBeam.getDistance(DistanceUnit.CM) < 2;
        } //TODO might need replace with actual beam breaker

        /** Sub Modes */
        //TODO Currently Cycling has wrapping, might need to implement hold system -_- for shortest path
        public void cycleRapidFire(){
            //Activates once if shots are available to shoot
            //Won't Cycle or Update Coutner if the robot has just shot something
            //Will only cycle if its empty at shooter spot
            //WILL NOT SPIN until the magnetic limit switch in in place (prevent skipping)
            //TODO Could still skip if mid spin it detects something, testing overwritten method for busy
            if (availableShots > 0 && !shotInProg && !isReadyShoot() && isCaroselInPlace() && !isBusy(caroselMotor)){//TODO Might not work
                cycleCaroselManual(); //TODO Heurstic, not efficent
                updateAvailableShots(); //TODO SLightly more inefficent but good for redundency
            }

            checkForShotOpportunity();
            checkResetTransfer();
            checkEndShotCoolDown();
        }
        public void cycleSortedFire(){

            //Activates once if shots are available to shoot
            //Won't Cycle or Update Coutner if the robot has just shot something
            //Will only cycle if its empty at shooter spot
            //WILL NOT SPIN until the magnetic limit switch in in place (prevent skipping)
            //TODO Could still skip if mid spin it detects something, testing overwritten method for busy
            if (availableShots > 0 && !shotInProg && !isReadyShoot() && isCaroselInPlace() && !isBusy(caroselMotor)){//TODO Might not work
                cyclePattern(sortedFireQueue);
                updateAvailableShots(); //TODO SLightly more inefficent but good for redundency
                sortedFireQueue ++;
            }

            checkForShotOpportunity();
            checkResetTransfer();
            checkEndShotCoolDown();

        }
        public void cycleCaroselManual(){
            cycleCarosel(getNextCycle()); //TODO Previous cycle too
        }

        /** Intake Cycling */
        public void autoIntakeCycle(){ //Added Magnetic Limitt Switch Here for saftey
            if (detectedArtifact() && !inventory[currentCycle].equals("Empty") && isCaroselInPlace()){
                updateInventory();
                cycleCaroselManual();
            }
        }

        /** Spindex */
        public void cycleCarosel(int desiredCycle){
            if (!transferCooldownActive && !shotInProg){
                updateInventory();
                currentCycle = desiredCycle;

                caroselMotor.setTargetPosition(RobotConstantsV2.caroselPos[desiredCycle]);
                caroselMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                caroselMotor.setVelocity(2400);
            }
        }
        public void cyclePattern(int number){
            int slot = getInvPosition(pattern[number]);

            if (slot == -1){
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

        /** Intake Color Detection */
        public float[] getHSV(){

            float[] hsv = new float[]{0F,0F,0F};
            Color.RGBToHSV(colorSensor.red()*255,colorSensor.green()*255,colorSensor.blue()*255, hsv);
            return hsv;
        }
        private boolean detectedArtifact(){
            return intakeBeam.getDistance(DistanceUnit.CM) < 3 || !getIntakeColor().equals("Empty");
        } //TODO ALSO ADD IF SPOT IN CURRCENT  CYCLE ISn"T EMPTY LOGIC WORK TOO
        public String getIntakeColor() {
            if (detectedArtifact()) {
                float[] HSV = getHSV();

                if (HSV[0] < 200){
                    return "Green";
                }
                else if (HSV[0] >= 200){
                    return "Purple";
                }
            }

            return "Empty";
        }

        /** Indicator Lights */
        public void updateIndicators(String mode, double disp, Turret turret){

            switch (mode){
                //TODO not synced to constants
                case("auto"):

                    //Motor Speed (Don't Need Multiple Colors)
                    if (turret.getTPSError(disp) > turret.getTPS(disp) * RobotConstantsV2.SHOOTER_MAX_SPEED_THRESHOLD || turret.getTPSError(disp) < turret.getTPS(disp) * RobotConstantsV2.SHOOTER_MIN_SPEED_THRESHOLD){
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

                    //double neededTPS; //TODO if separate TPS from angle, need use this
//                    //Code for Long
//                    if (disp == -1){
//                        neededTPS = RobotConstantsV2.FAR_TPS;
//                    }
//
//                    //Code for Close
//                    else if (disp == -2){
//                        neededTPS =
//                    }
//
                    //Input constant disp in main file
                    //Motor Speed (Don't Need Multiple Colors) //TODO TPS ERror Going to be wrong here (not right)
                    if (turret.getTPSError(disp) > turret.getTPS(disp) * RobotConstantsV2.SHOOTER_MAX_SPEED_THRESHOLD || turret.getTPSError(disp) < turret.getTPS(disp) * RobotConstantsV2.SHOOTER_MIN_SPEED_THRESHOLD){
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

                case("humanIntake"):

                    indicatorOne.setPosition(RobotConstantsV2.INDICATOR_BLUE); //Shows that Human Player Mode

                    //Double Check Make Sure it empty currently
                    if (!isEmptySpot()){
                        //Inventory is Full
                        indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_RED);
                    }
                    else if (!detectedArtifact() || inventory[currentCycle].equals("Empty")){
                        indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_GREEN);
                    }
                    else{
                        indicatorTwo.setPosition(RobotConstantsV2.INDICATOR_BLUE);
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
            return !magLS.getState();
        }

        /** Final Updates */
        public void telemetryCarosel(){
            telemetry.addLine(String.format("Inventory: (%s, %s, %s)", inventory[0], inventory[1], inventory[2]));
            float[] HSV = getHSV();
            telemetry.addLine(String.format("HSV: (%0.5f, %0.5f, %0.5f)", HSV[0], HSV[1], HSV[2]));
        }
    }
}