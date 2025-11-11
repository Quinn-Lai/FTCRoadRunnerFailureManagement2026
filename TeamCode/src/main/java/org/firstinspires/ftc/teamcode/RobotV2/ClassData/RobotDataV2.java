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

//import kotlin.random.FallbackThreadLocalRandom;


//Class to Organize each robot part and store the objects of each part of the robot
//Should be imported into RoadRunnerData to utilize each component
public class RobotDataV2 {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private static ColorRange startColor;
    private static boolean startedLeft;
    private static ElapsedTime runtime;
    private ElapsedTime delay;
    private boolean pendingColor;
    private boolean pendingPosition;

    //Sub Classes
    private DriveTrain driveTrain;
    private Turret turret;
    private Carosel carosel;
    private AprilTagVisionV2 atData;
    //OpenCV
    private boolean openCVEnabled;

    //----------------------------------------

    //Constructor
    public RobotDataV2(HardwareMap hardwareMap, Telemetry telemetry, AprilTagVisionV2 atData){

        //OpenCV
        //startColor = ColorRange.RED; //Default in Case Auto Wasn't Run First

        //Turret

        this.hardwareMap = hardwareMap;
        turret = new Turret(hardwareMap);
        carosel = new Carosel(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap);

        this.atData = atData;
        this.telemetry = telemetry;
        delay = new ElapsedTime();

        //Intake

        //Carosel

        runtime = null;
    }

    //----------------------------------------

    //Main Sub Classes

    public Carosel getCarosel(){
        return carosel;
    }
    public DriveTrain getDriveTrain(){
        return driveTrain;
    }
    public Turret getTurret() {
        return turret;
    }

    //----------------------------------------


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
    public void setOpenCVEnabled(boolean openCVEnabled) {
        this.openCVEnabled = openCVEnabled;
    }
    public static ColorRange getStartColor() {
        return startColor;
    }

    public static void setStartColor(ColorRange color) {
        startColor = color;
    }
    public static boolean getStartedLeft() {
        return startedLeft;
    }
    public static void setStartedLeft(boolean left) {
        startedLeft = left;
    }
    public static String getStartingPosition(){
        if (startedLeft){
            return "Left Side";
        }
        else{
            return "Right Side";
        }
    }
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }
    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
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
    public void delayOrder(int delay) {
        this.delay.reset();
        while (this.delay.milliseconds() < delay) {
            getDriveTrain().omniDrive();
        }
    }
    public void updateTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    //----------------------------------------

    public class DriveTrain{

        //Drive Train
        private DcMotor LFmotor;
        private DcMotor RFmotor;
        private DcMotor LBmotor;
        private DcMotor RBmotor;

        //Sensors
        private IMU imu;

        //Driver
        private boolean robotCentric;
        private Gamepad mode;
        private String tag;
        private boolean defaultToAutoAim;

        public DriveTrain(HardwareMap hardwareMap){
            //Basic Motors
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

            robotCentric = true;
            openCVEnabled = false;
            pendingColor = true;
            pendingPosition = true;
            defaultToAutoAim = true;

            if (!robotCentric){
                imu = hardwareMap.get(IMU.class, "imu");
                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
                imu.initialize(parameters);
            }
        }

        //----------------------------------------

        //Mode

        public Gamepad getMode() {
            return mode;
        }
        public String getTag(){
            return tag;
        }
        public void setMode(Gamepad mainDriver, String tag) {

            //Only Changes Drivers if the person trying to swap isn't already the main driver
            //if (!(this.mainDriver.equals(mainDriver))){
            //this.subDriver = this.mainDriver;   //current main driver becomes P2
            this.mode = mainDriver;       //new main driver becomes selected player
            this.tag = tag;
            this.mode.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            //this.subDriver.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
            //}
        }
        public void switchMode(){

            if (defaultToAutoAim){
                defaultToAutoAim = false;
            }

            else{
                defaultToAutoAim = true;
            }
        }
        public boolean getCurrentMode(){
            return defaultToAutoAim;
        }
        public void updateModeColor(){
            if (defaultToAutoAim){
                    mode.setLedColor( 0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);

                }
            else{
                    mode.setLedColor( 0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
                }
        }

        //----------------------------------------

        //DT

        public void omniDrive(){

            double x = mode.left_stick_x;
            double y = -mode.left_stick_y;
            double z = mode.right_stick_x;

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
        }

        //----------------------------------------

        //Feedback

        public void checkEndgame(){
            double currentRuntime = getRuntime();

            if (currentRuntime >= 90 && currentRuntime <= 92){
                mode.rumble(2);
                mode.setLedColor( 1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                telemetry.addLine("END GAME!");
            }

            else if (currentRuntime >= 110 && currentRuntime <= 120){
                mode.rumble(2);
                double time = 120 - currentRuntime;
                telemetry.addLine(Math.floor(time) + " Seconds Left!");
            }
        }
        public void displayTelemetryData(){
            telemetry.addData("Time: ", Math.floor(getRuntime()) + " Seconds");
            telemetry.addData("Main Driver: ", tag);
        }

        //----------------------------------------

        //Misc

        public DcMotor getLFmotor(){
            return LFmotor;
        }

        //----------------------------------------

    }
    public class PIDControl{
        private double kP = 6.7;
        private double kI = 0.53;
        private double kD = 0.3;
        private ElapsedTime PIDTimer = new ElapsedTime();
        private double integralSum = 0;
        private double lastError = 0;
        public String getPIDCos(){
            return kP + ", " + kI + ", " + kD;
        }

        //----------------------------------------

        //PID

        public double PIDShooter(double current, double desired) {

            double currentTime = PIDTimer.seconds();

            double output = 0;

            double error = desired - current;

            integralSum += error * currentTime;

            double derivate = (error - lastError) / currentTime;

            lastError = error;

            PIDTimer.reset();

            output = (error * kP) + (derivate * kD) + (integralSum * kI);

            return output;
        }
    }
    public class Turret extends PIDControl{

        //Motors
        public DcMotorEx shooterMotor;
        private Servo shooterServo;

        //Physics Params
        private final double a = -9.8; //Acceleration m/s^2
        public double heightOfLauncher = 0.3556; //Meters, need compensate for height of launcher
        private final double height = 1.2 - heightOfLauncher; //Meters 1.05
        private final double vY = Math.sqrt(2 * -a * height);
        private double vX = 0;
        private double vX0 = 0; //initial vX of the robot at an instant in time

        //Motor Params
        private final double spinWheelRadius = 0.096/2; //Meters
        private final double w = 28.0; //Ticks
        private boolean toggleTurretAim;
        private boolean toggleTurretManualClose;

        private ElapsedTime scrimTimerAuto;
//        private final double dragX = 1.67;
//        private final double dragY = 1.1;

        //----------------------------------------

        private Turret(HardwareMap hardwareMap){
            scrimTimerAuto = new ElapsedTime();

            shooterMotor = (DcMotorEx)hardwareMap.get(DcMotor.class,"shooterMotor"); //temp
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            shooterMotor.setDirection(DcMotor.Direction.REVERSE);

            shooterServo = hardwareMap.get(Servo.class,"shooterServo");
            toggleTurretAim = false;
            toggleTurretManualClose = false;

        }

        //----------------------------------------

        //Turret Mode

        public void switchTurretMode(){
            if (toggleTurretAim){
                toggleTurretAim = false;
                toggleTurretManualClose = false;
            }

            else{
                toggleTurretAim = true;
                toggleTurretManualClose = false;
            }
        }
        public boolean isToggleTurretAim(){
            return toggleTurretAim;
        }

        public void switchTurretManualClose(){
            if (toggleTurretManualClose){
                toggleTurretManualClose = false;
                toggleTurretAim = false;
            }

            else{
                toggleTurretManualClose = true;
                toggleTurretAim = false;
            }
        }
        public boolean isToggleTurretManualClose(){
            return toggleTurretManualClose;
        }

        public void reverseShooterMotor(){
            shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        public void unreverseShooterMotor(){
            shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        //----------------------------------------

        //Physics Calculations

        private double getYEquation(double time){
            return vY * time + 0.5 * a * Math.pow(time, 2);
        }
        private double getXEquation(double time){
            return vX * time;
        }
        public double getHeightOfLauncher(){
            return heightOfLauncher;
        }

        //Using the formula -b/2a, SAME THING as derivative of position equation and the vf = vi + at equation
        private double getTimeExtrema(){
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
            return Math.sqrt(Math.pow(getVy(),2) + Math.pow(getVx(disp),2)) * RobotConstantsV2.yDrag;
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
            return (getRadiansPerSecond(disp) * getW()) / (2 * Math.PI);
        }

        //----------------------------------------

        //Shooter

        public void powerShooterMotor(double TPS){

            double leftCurrent = shooterMotor.getVelocity();

            shooterMotor.setVelocity(PIDShooter(leftCurrent,TPS));
        }

        public void killShooterPower(){
            shooterMotor.setVelocity(0);
        }

        //----------------------------------------

        //Hood Angle

        private double convertDegToServo(double angle){
            return -0.031506 * angle + 1.99244;
        }
        public void angleRobot(double disp){

            //double desiredAngle = getAngleTotal(disp)+15;
            double desiredAngle = getAngleTotal(disp) + 15;
            shooterServo.setPosition(convertDegToServo(desiredAngle));

            //Servo Movement

            //servo.setPosition(convertDegToServo(desiredAngle));

        }

        //----------------------------------------

        //Misc
        public void aimBall(double disp){
            angleRobot(disp);
            double TPS = getTPS(disp);
            //powerShooterMotor(TPS);

            //Scuffed Logic Here (Switch it)
            if (disp < 2.5){
                //powerShooterMotor(TPS);
                shooterMotor.setVelocity(TPS);
            }
            else{
                shooterMotor.setVelocity(RobotConstantsV2.longTPS);
                //powerShooterMotor(constants.longTPS);
            }
        }

        public void aimBall(double disp, boolean z){
            angleRobot(disp);
            double TPS = getTPS(disp);
            //powerShooterMotor(TPS);

            //Scuffed Logic Here (Switch it)
            if (disp < 2.5){
                powerShooterMotor(TPS);
                //shooterMotor.setVelocity(TPS);
            }
            else{
                //shooterMotor.setVelocity(constants.longTPS);
                powerShooterMotor(RobotConstantsV2.longTPS);
            }
        }
        public void telemetryArm(double disp){
            telemetry.addLine("Kinematics: \n");

            telemetry.addData("Total Velocity: ", getVelocityTotal(disp));
            telemetry.addData("vY: ", getVy());
            telemetry.addData("vX: ",getVx(disp));
            telemetry.addData("vX0: ", getInitalVelocityX());
            telemetry.addData("Drag Vx: ", RobotConstantsV2.xDrag);
            telemetry.addData("Drag Vy: ", RobotConstantsV2.yDrag);
            telemetry.addLine("-------------------------- \n");

            telemetry.addLine("TPS: \n");

            telemetry.addData("True TPS: ", shooterMotor.getVelocity());
            telemetry.addData("Ticks Per Second: ", getTPS(disp));
            telemetry.addData("TPS Error:", shooterMotor.getVelocity() - getTPS(disp));

            telemetry.addLine("-------------------------- \n");

            telemetry.addLine("Outputs: \n");

            telemetry.addData("Time Extrema:", getTimeExtrema());
            telemetry.addData("Displacement (m): ", disp);
            telemetry.addData("Angle: " , getAngleTotal(disp));
            telemetry.addData("True Servo Angle:",convertDegToServo(getAngleTotal(disp)));

        }
    }
    public class Carosel{

        //Hardware
        private DcMotorEx intakeMotor;
        private DcMotorEx caroselMotor;
        private Servo transferServo;
        private Servo indicatorOne;
        private Servo indicatorTwo;
        private DigitalChannel magLS;
        private ColorSensor colorSensor;
        private DistanceSensor colorSensorDist;
        private DistanceSensor transferBeam;

        //Storage
        private String[] inventory;
        private String[] pattern;
        private int currentCycle;

        //Intake
        private boolean intakeMotorOn;

        //Transfer
        private boolean transferUp;

        //Modes
        private boolean rapidFireOn;
        private boolean shotInProg;
        private boolean transferCooldownActive;
        private ElapsedTime transferCooldown;
        private int availableShots;

        //----------------------------------------

        public Carosel(HardwareMap hardwareMap){

            intakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class,"intakeMotor");
            caroselMotor = (DcMotorEx) hardwareMap.get(DcMotor.class,"caroselMotor");
            transferServo = hardwareMap.get(Servo.class,"transferServo");

            indicatorOne = hardwareMap.get(Servo.class,"indicatorOne");
            indicatorTwo = hardwareMap.get(Servo.class,"indicatorTwo");

            magLS = hardwareMap.get(DigitalChannel.class,"magLS");
            colorSensor = hardwareMap.get(ColorSensor.class,"colorSensor");
            colorSensorDist = hardwareMap.get(DistanceSensor.class,"colorSensor");

            transferBeam = hardwareMap.get(DistanceSensor.class,"transferBeam");

            inventory = new String[]{"Empty","Empty","Empty"};
            pattern = new String[]{"Empty","Empty","Empty"};

            //Boolean Standardization

            intakeMotorOn = false;
            rapidFireOn = false;
            transferUp = false;
            shotInProg = false;
            transferCooldownActive = false;

            transferCooldown = new ElapsedTime();
            availableShots = 0;

        }

        //----------------------------------------

        //Autonomous
        public void setInventoryAuto(){
            inventory = new String[]{"Green","Purple","Purple"};
        }


        //----------------------------------------

        //Intake

        public void intakeArtifact(){
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

        //----------------------------------------

        //General Use
        public void updatePattern(String[] pattern){
            this.pattern = pattern;
        }
        private boolean hasPattern() {
            return Arrays.equals(inventory,pattern);
        }

        //----------------------------------------

        //Spindex
//TODO Add human intake feature
        public void activateRapidFire(){
            updateAvailableShots();
            rapidFireOn = true;
        }
        public boolean isRapidFireOnMode(){
            return rapidFireOn;
        } //TODO Reverse if Automatic
        private void checkForShotOpportunity(){
            if (isCaroselInPlace() && rapidFireOn && !shotInProg && isReadyShoot()){ //TODO There is nothing to prevent an accdiental mag ls read from killing everything
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
        //TODO Currently Cycling has wrapping, might need to implement hold system -_- for shortest path
        //Mode
        public void cycleRapid(){
            if (rapidFireOn) { //TODO Use case switch here

                //Activates once if shots are available to shoot
                //Won't Cycle or Update Coutner if the robot has just shot something
                //Will only cycle if its empty at shooter spot
                //WILL NOT SPIN until the magnetic limit switch in in place (prevent skipping)
                //TODO Could still skip if mid spin it detects something, could try is busy but that is buggy
                if (availableShots > 0 && !shotInProg && !isReadyShoot() && isCaroselInPlace() && !caroselMotor.isBusy()){//TODO Might not work
                    cycleCarosel(getNextCycle());
                    updateAvailableShots(); //TODO SLightly more inefficent but good for redundency
                }

                else{
                    rapidFireOn = false;
                }

                checkForShotOpportunity();
                checkResetTransfer();
                checkEndShotCoolDown();
            }
        }

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

        //TODO Rapid Fire and pattern fire

        //----------------------------------------

        //Inventory

        private void updateAvailableShots(){

            int shotCounter = 0;

            for (String i:inventory){
                if (i.equals("Green") || i.equals("Purple")){
                    shotCounter ++;
                }
            }

            availableShots = shotCounter;
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
        public boolean isReadyShoot(){
            return !inventory[currentCycle].equals("Empty");
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
        private String getTransferColor(){ //TODO REDO WITH NEW 4th POS
           return inventory[currentCycle];
        }
        private boolean checkIfEmptySpot(){
            for (String i: inventory){
                if (i.equals("Empty")){
                    return true;
                }
            }
            return false;
        }
        public void telemetryInventory(){
            telemetry.addData("Inventory: ", inventory[0] + ", " + inventory[1] + ", " + inventory[2]);
        }
        private void updateInventory(){

            //Won't override updates
            if (inventory[currentCycle].equals("Green") || inventory[currentCycle].equals("Purple")){
                return;
            }

            inventory[currentCycle] = getIntakeColor();
        }

        //----------------------------------------

        //Color Sensor

        public float[] getHSV(){

            float[] hsv = new float[]{0F,0F,0F};

            Color.RGBToHSV(colorSensor.red()*255,colorSensor.green()*255,colorSensor.blue()*255, hsv);

            return hsv;
        }
        private boolean detectedArtifact(){
            return colorSensorDist.getDistance(DistanceUnit.CM) < 3;
        }
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
        public void telemetryColorSensor(){
            float[] HSV = getHSV();
            telemetry.addData("HSV", HSV[0] + "," + HSV[1] + "," + HSV[2]);
        }

        //----------------------------------------

        //Indicator Lights

        public void updateIndicators(){
            if (hasPattern()){ //TODO Set Numbers
                indicatorOne.setPosition(1);
            }
            else{
                indicatorOne.setPosition(0);
            }

            if (getTransferColor().equals("Green")){
                indicatorTwo.setPosition(1);
            }
            else if (getTransferColor().equals("Purple")){
                indicatorTwo.setPosition(0);
            }
            else{
                indicatorTwo.setPosition(0.5);
            }
        }

        //----------------------------------------

        //Transfer

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
        public boolean isShotSuccess(){
            return transferBeam.getDistance(DistanceUnit.CM) < 2;
        } //TODO might need replace with actual beam breaker


        //----------------------------------------

        //Magnetic Limit Swtich

        public boolean isCaroselInPlace(){
            return !magLS.getState();
        }

    }
}