package org.firstinspires.ftc.teamcode.ClassData;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

//import kotlin.random.FallbackThreadLocalRandom;


//Class to Organize each robot part and store the objects of each part of the robot
//Should be imported into RoadRunnerData to utilize each component
public class RobotData{
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
    private AprilTagVision atData;
    private RobotConstants constants;

    //OpenCV
    private boolean openCVEnabled;

    //Constructor
    public RobotData(HardwareMap hardwareMap, Telemetry telemetry, AprilTagVision atData){

        //OpenCV
        //startColor = ColorRange.RED; //Default in Case Auto Wasn't Run First

        //Turret

        this.hardwareMap = hardwareMap;
        turret = new Turret(hardwareMap);
        carosel = new Carosel(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap);
        constants = new RobotConstants();

        this.atData = atData;
        this.telemetry = telemetry;
        delay = new ElapsedTime();


        //Intake

        //Carosel

        runtime = null;
    }

    public Carosel getCarosel(){
        return carosel;
    }
    public DriveTrain getDriveTrain(){
        return driveTrain;
    }
    public Turret getTurret() {
        return turret;
    }
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

        //Booleans
        private boolean robotCentric;

        //Driver
        private Gamepad mode;
        private String tag;
        private boolean defaultToAutoAim;
        private boolean endGameActive;

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
            endGameActive = false;

            if (!robotCentric){
                imu = hardwareMap.get(IMU.class, "imu");
                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
                imu.initialize(parameters);
            }
        }

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

            if (!endGameActive){

                if (defaultToAutoAim){
                    mode.setLedColor( 0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);

                }

                else{
                    mode.setLedColor( 0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
                }


            }

        }

        //Feedback
        public void checkEndgame(){
            double currentRuntime = getRuntime();

            if (currentRuntime >= 90 && currentRuntime <= 92){

                endGameActive = true;

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

        public DcMotor getLFmotor(){
            return LFmotor;
        }

        //Drive Code
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

    }
    public class PIDControl{
//        private double kP = 5.5;
//
//        private double kI = 0.015;
//        private double kD = 0.5;
        private double kP = 0.79;

        private double kI = 0.222;
        private double kD = 0.01;

        private ElapsedTime PIDTimer = new ElapsedTime();

        private double integralSum = 0;
        private double lastError = 0;

        public String getPIDCos(){
            return kP + ", " + kI + ", " + kD;
        }

        public void changeKp(double val){
            kP += val;
        }

        public void changeKI(double val){
            kI += val;
        }

        public void changeKD(double val){
            kD += val;
        }

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
        private CRServo spinnerServo;

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
//        private final double dragX = 1.67;
//        private final double dragY = 1.1;

        private Turret(HardwareMap hardwareMap){
            shooterMotor = (DcMotorEx)hardwareMap.get(DcMotor.class,"shooterMotor"); //temp
            spinnerServo = hardwareMap.get(CRServo.class, "spinnerServo");
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            shooterMotor.setDirection(DcMotor.Direction.REVERSE);

            shooterServo = hardwareMap.get(Servo.class,"shooterServo");
            toggleTurretAim = false;
        }

        public void switchTurretMode(){
            if (toggleTurretAim){
                toggleTurretAim = false;
            }

            else{
                toggleTurretAim = true;
            }
        }
        public boolean isToggleTurretAim(){
            return toggleTurretAim;
        }


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
            return Math.sqrt(Math.pow(getVy(),2) + Math.pow(getVx(disp),2)) * RobotConstants.yDrag;
        }

        private double getAngleTotal(double disp){
            return Math.atan2(getVy(),getVx(disp)) * (180 / Math.PI);
        }

        //Motor Conversions

        private double getSpinWheelRadius(){
            return spinWheelRadius;
        }

        private double getRadiansPerSecond(double disp){ //From Velocity
            return getVelocityTotal(disp)/getSpinWheelRadius();
        }

        //Useless just for Cosmetics
        private double getDesiredRPM(double disp){ //From Radians Per SEcond
            return getRadiansPerSecond(disp) * 60 / (2 * Math.PI);
        }

        private double getW(){
            return w;
        }

        private double getTPS(double disp){ //From Radians per Second
            return (getRadiansPerSecond(disp) * getW()) / (2 * Math.PI);
        }

        //Shooter Actions

        public void powerShooterMotor(double TPS){

            double leftCurrent = shooterMotor.getVelocity();

            shooterMotor.setVelocity(PIDShooter(leftCurrent,TPS));
        }

        public void killShooterPower(){
            shooterMotor.setVelocity(0);
        }

        private double convertDegToServo(double angle){
            return -0.031506 * angle + 1.99244;
        }

        private void angleRobot(double disp){

            //double desiredAngle = getAngleTotal(disp)+15;
            double desiredAngle = getAngleTotal(disp) + 15;
            shooterServo.setPosition(convertDegToServo(desiredAngle));

            //Servo Movement

            //servo.setPosition(convertDegToServo(desiredAngle));

        }

        public void centerShot(double x, boolean e){

            if (x < -5){
                spinnerServo.setPower(0.1);
            }

            else if (x > 5){
                spinnerServo.setPower(-0.1);
            }

            else{
                spinnerServo.setPower(0);
            }

        }

        public void angleLeftSpin(){
            spinnerServo.setPower(-0.15);
        }

        public void angleRightSpin(){
            spinnerServo.setPower(0.15);
        }


        public void centerShot(double yaw){

             if (yaw < -0.1){
                spinnerServo.setPower(1);
            }

            else if (yaw > 0.1){
                spinnerServo.setPower(-1);
            }

            else{
                spinnerServo.setPower(0);
            }

        }

        public void killSpinnerServo(){
            spinnerServo.setPower(0);
        }

        public void aimBall(double disp){
            angleRobot(disp);
            double TPS = getTPS(disp);
            //powerShooterMotor(TPS);

            //Scuffed Logic Here (Switch it)
            if (disp < 2.5){
                powerShooterMotor(TPS);
                //shooterMotor.setVelocity(TPS);
            }
            else{
                shooterMotor.setVelocity(constants.longTPS);
                //powerShooterMotor(constants.longTPS);
            }
        }

        public void telemetryArm(double disp){
            telemetry.addLine("Kinematics: \n");

            telemetry.addData("Total Velocity: ", getVelocityTotal(disp));
            telemetry.addData("vY: ", getVy());
            telemetry.addData("vX: ",getVx(disp));
            telemetry.addData("vX0: ", getInitalVelocityX());
            telemetry.addData("Drag Vx: ", RobotConstants.xDrag);
            telemetry.addData("Drag Vy: ", RobotConstants.yDrag);
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

        private Servo elevatorServo;
        private boolean elevatorTop;
        private DigitalChannel magneticLS;
        private DigitalChannel touchSensor;
        private DcMotorEx caroselMotor;
        private ColorSensor csIntake;
        private DistanceSensor elevatorBeam;
        private DistanceSensor csIntakeDist;
        private String[] inventory;
        private CRServo intakeServo;
        private boolean intakeServoOn;
        //FailSafe Timers
        private ElapsedTime elevatorTimer;
        private ElapsedTime cycleTimer;

        //Detection
        private String[] pattern;

        //private String[] storage;

        //Cycling
        private int currentCaroselGlobalPos;
        private int currentCycle;
        private boolean cycleInProg;
        private boolean artifactDetected;

        public Carosel(HardwareMap hardwareMap){

            artifactDetected = false;
            cycleInProg = false;
            currentCycle = 0;
            currentCaroselGlobalPos = 0;
            elevatorTimer = new ElapsedTime();
            cycleTimer = new ElapsedTime();

            elevatorTop = false;
            elevatorServo = hardwareMap.get(Servo.class,"elevatorServo");
            elevatorBeam = hardwareMap.get(DistanceSensor.class,"elevatorBeam");

            magneticLS = hardwareMap.get(DigitalChannel.class,"magneticLS");
            caroselMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "caroselMotor");
            caroselMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            caroselMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            caroselMotor.setDirection(DcMotorEx.Direction.REVERSE);
            caroselMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            elevatorServo = hardwareMap.get(Servo.class,"elevatorServo");

            csIntake = hardwareMap.get(ColorSensor.class,"csIntake");
            csIntakeDist = hardwareMap.get(DistanceSensor.class,"csIntake");
            touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");

            magneticLS.setMode(DigitalChannel.Mode.INPUT);

            inventory = new String[]{"Empty","Empty","Empty"};

            intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
            intakeServoOn = false;
        }

        //Intake
        //Could do boolean based or activation based (action based here)

        public void powerIntakeServo(double power){

            intakeServo.setPower(power);
        }

        public void shiftElevator(){

            if (!elevatorTop){
                elevatorServo.setPosition(constants.ELEVATOR_TOP);
                elevatorTop = true;
            }

            else{
                elevatorServo.setPosition(constants.ELEVATOR_BOT);
                elevatorTop = false;
            }

        }

        public void ariseElevator(){
            elevatorServo.setPosition(constants.ELEVATOR_TOP);
            elevatorTop = true;
            elevatorTimer.reset();
        }

        public void checkDeriseElevator(){
            if (elevatorTop && (elevatorTimer.milliseconds() > 1500 || isElevatorTop())){
                elevatorServo.setPosition(constants.ELEVATOR_BOT);
                elevatorTop = false;
            }
        }

        private boolean isElevatorTop(){
            return elevatorBeam.getDistance(DistanceUnit.CM) < 5;
        }

        public void killIntakeServo(){
            intakeServo.setPower(constants.INTAKE_OFF);
            intakeServoOn = true;
        }

        public void setCaroselPosition(int tick, Gamepad gamepad){
            if (!elevatorTop){
                ElapsedTime temp = new ElapsedTime();

                caroselMotor.setTargetPosition(tick);
                caroselMotor.setVelocity(1500);
                caroselMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                temp.reset();

                while (caroselMotor.isBusy()){

                    double time = temp.milliseconds();

                    getDriveTrain().omniDrive(gamepad);

                    if (time > 1000){
                        break;
                    }

                }

                caroselMotor.setVelocity(0);
            }

        }

        private boolean checkIfEmptySpot(){
            for (String i: inventory){
                if (i.equals("Empty")){
                    return true;
                }
            }
            return false;
        }

        public void printInv(){
            telemetry.addData("Inventory: ", inventory[0] + ", " + inventory[1] + ", " + inventory[2]);
        }

        public void updateCaroselReaction(){
            if (detectedArtifact() && checkIfEmptySpot()){
                artifactDetected = true;
                startCycling(getClosestEmpty());
            }
        }

        //purposely need to create a subzone

        public boolean getArtifactDetected(){
            return artifactDetected;
        }

//        public void cyclePositionalSpot(int pos, Gamepad gamepad){
//
//            currentCaroselPos = pos;
//
//            if (pos == 0){
//                setCaroselPosition(constants.caroselPos[0], gamepad);
//            }
//
//            else if (pos == 1){
//                setCaroselPosition(constants.caroselPos[1], gamepad);
//            }
//
//            else if (pos == 2){
//                setCaroselPosition(constants.caroselPos[2], gamepad);
//            }
//
//        }

        public void zeroCaroselMotor(){
            caroselMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            currentCaroselGlobalPos = 0;
            currentCycle = 0;
        }

        public void incCaroselMotor(){
            currentCaroselGlobalPos += 5;
        }

        public void killCaroselMotor(){
            caroselMotor.setVelocity(0);
        }

        private int getClosestEmpty(){

            if (inventory[currentCycle].equals("Empty")){
                return currentCycle;
            }
            else if (inventory[incrementCurrentPos(1)].equals("Empty")) {
                return incrementCurrentPos(1);
            }
            else if (inventory[incrementCurrentPos(2)].equals("Empty")) {
                return incrementCurrentPos(2);
            }

            return -1;

        }

        //For Manual Control

        public int getCurrentCycle(){
            return currentCycle;
        }

        private int getCycleError(int desiredCycle){

            telemetry.addData("PosTESTTHIS",desiredCycle - currentCycle);
            if (desiredCycle < currentCycle){

                //Limited Case Structure
                if (Math.abs(currentCycle - desiredCycle) == 1){
                    return 2;
                }

                else{
                    return 1;
                }

            }

            else if (desiredCycle > currentCycle){
                telemetry.addData("Pos1",desiredCycle - currentCycle);
                return desiredCycle - currentCycle;
            }

            else{
                return 0;
            }
        }

//        public void cycleCarosel(int desiredCycle, Gamepad gamepad){
//            if (!elevatorTop){
//
//                intakeServo.setPower(constants.INTAKE_POWER);
//
//                int error = getCycleError(desiredCycle);
//
//                currentCycle = desiredCycle;
//                ElapsedTime temp = new ElapsedTime();
//
//                currentCaroselGlobalPos = constants.caroselIncrement * error + currentCaroselGlobalPos;
//
//                caroselMotor.setTargetPosition(currentCaroselGlobalPos);
//                caroselMotor.setVelocity(1500);
//                caroselMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//                temp.reset();
//
//                while (caroselMotor.isBusy()){
//
//                    double time = temp.milliseconds();
//
//                    getDriveTrain().omniDrive(gamepad);
//
//                    if (time > 1000){
//                        break;
//                    }
//
//                }
//
//                caroselMotor.setVelocity(0);
//
//                intakeServo.setPower(constants.INTAKE_OFF);
//            }
//        }

//        public void cycleInventory(int cycleNumber){
//            for (int count = 0; count < cycleNumber; count++){
//                String temp = inventory[2];
//                inventory[2] = inventory[1];
//                inventory[1] = inventory[0];
//                inventory[0] = temp;
//            }
//        }

        private void updateInventory(int desiredCycle){
            inventory[desiredCycle] = getColorIntake();
        }

        public void startCycling(int desiredCycle){
            if (!elevatorTop){
                int error = getCycleError(desiredCycle);
                updateInventory(currentCycle);
                currentCycle = desiredCycle;
                //cycleInventory(error);

                currentCaroselGlobalPos = constants.caroselIncrement * error + currentCaroselGlobalPos;
                telemetry.addData("Position Run To: ", currentCaroselGlobalPos);

                caroselMotor.setTargetPosition(currentCaroselGlobalPos);
                caroselMotor.setVelocity(500);
                caroselMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                cycleTimer.reset();
                cycleInProg = true;
            }
        }

        public void checkCycleEnd(){

            if (caroselMotor.isBusy() && cycleInProg){
                intakeServo.setPower(constants.INTAKE_POWER);
                intakeServoOn = true;

                if (cycleTimer.milliseconds() > 1000){
                    telemetry.addLine("test");
                    intakeServo.setPower(constants.INTAKE_OFF);
                    intakeServoOn = false;
                    cycleInProg = false;
                }

            }

            else{
                //caroselMotor.setVelocity(0);
                if (cycleInProg){
                    telemetry.addLine("test");
                    intakeServo.setPower(constants.INTAKE_OFF);
                    intakeServoOn = false;
                    cycleInProg = false;
                }
                artifactDetected = false; //For Auto Aim ends subzone
            }
        }

        public int incrementCurrentPos(int inc){

            int nextPos = currentCycle + inc;

            if (nextPos > 2){

                if (inc == 1){
                    nextPos = 0;
                }

                else {
                    nextPos = 1;
                }
            }

            return nextPos;

        }

        public void shootPattern(double disp, double angle){

        }
        public void powerCaroselMotor(int power){
            if (!elevatorTop){
                caroselMotor.setVelocity(power);
            }
        }

        public void killCaroselPower(){
            caroselMotor.setVelocity(0);
        }

        private boolean elevatorAtRest(){
            return touchSensor.getState();
        }

        private boolean detectedArtifact(){
            float[] HSV = getHSV();

            return csIntakeDist.getDistance(DistanceUnit.CM) < 3 || HSV[2] < 100;
        }

        public void intakeArtifact(){

            if (!intakeServoOn){
                intakeServo.setPower(constants.INTAKE_POWER);
                intakeServoOn = true;
            }

            else{
                intakeServo.setPower(constants.INTAKE_OFF);
                intakeServoOn = false;
            }

        }

        public void ejectArtifact(){
            if (intakeServoOn){
                intakeServo.setPower(-constants.INTAKE_POWER);
                intakeServoOn = false;
            }

            else{
                intakeServo.setPower(constants.INTAKE_OFF);
                intakeServoOn = true;
            }
        }

        public void forceStopArtifact(){
            intakeServo.setPower(constants.INTAKE_OFF);
            intakeServoOn = true;
        }


        //High (Sense Magnet) = true; reverse false
        public boolean getMagnetState(){
            return !magneticLS.getState();
        }

        public float[] getHSV(){

            /*
            double redNorm = (double)(red)/255;
            double greenNorm = (double)(green)/255;
            double blueNorm = (double)(blue)/255;

            double maxC = Math.max(redNorm,Math.max(greenNorm,blueNorm));
            double minC = Math.min(redNorm,Math.min(greenNorm,blueNorm));

            double h = 0;
            double s = 0;
            double v = 0;

            //Value
            v = maxC;

            //Saturation

            if (maxC == 0){
                s = 0;
            }

            else{
                s = (maxC - minC) / maxC;
            }

            //Hue

            if (maxC == minC){
                h = 0;
            }

            else if (maxC == redNorm){
                h = ((greenNorm - blueNorm) / (maxC - minC)) % 6;
            }

            else if (maxC == greenNorm){
                h = 2 + (blueNorm - redNorm) / (maxC - minC);
            }

            else if (maxC == blueNorm){
                h = 4 + (red - green) / (maxC - minC);
            }

            h *= 60;

            if (h<360){
                h += 360;
            }
            */

            float[] hsv = new float[]{0F,0F,0F};

            Color.RGBToHSV(csIntake.red()*255,csIntake.green()*255,csIntake.blue()*255, hsv);

            return hsv;

            //return new Double[]{h,s,v};


        }

//        private double getS(int red, int green, int blue){
//
//        }
//
//        private double getV(int red, int green, int blue){
//
//        }


        public String getColorIntake() {

            /*
            //Purple
            if (csIntake.red() > 100 && csIntake.blue() > 100){
                return "Purple";
            }

            //Green
            else if (csIntake.green() > 100){
                return "Green";
            }

            else{
                return null;
            }
             */
            if (detectedArtifact()) {
                float[] HSV = getHSV();

                telemetry.addData("HSV", HSV[0] + "," + HSV[1] + "," + HSV[2]);

                if (HSV[0] < 200){
                    return "Green";
                }

                else if (HSV[0] >= 200){
                    return "Purple";
                }

            }

            return "Empty";
        }



        public ColorSensor getCS(){
           return csIntake;
        }

        /*
        public void cycleCarosel(double power, int numCycles){

            if (elevatorAtRest()){
                boolean state = getMagnetState();
                //boolean nextState = !state; //Might need fact check this

                //boolean passed = false;
                //boolean reachedGoal = false;
                //Boolean[] cycleCache = new Boolean[]{state,nextState}; //If decide, could check for future state
                boolean cycleCache = state;
                int cycleCount = 0;
                String temp = "";

                //Keep Moving Circle while it hasn't passed by the empty space once or state is not on a magnet
                //Convert to global loop
                //while ((!passed || !reachedGoal) && !abortNow){
                while (!getDriveTrain().getAbortStatus()){
                    getDriveTrain().abortCheck();
                    state = getMagnetState();

                    getDriveTrain().omniDrive();
                    caroselMotor.setPower(power);

                    //Passed between right now (Doesn't detect)
                    //Guarantees that it cycles to the very next instant with the magnet
                    //if (!state) passed = true; // passed magnet on to magnet off
                    //passed might not be nessessary anymore with new system


                    //Checks if the current state has changed from the inital position
                    //Counts every time the magnet changes
                    if (state != cycleCache){

                        if (power > 0){
                            temp = inventory[2];

                            inventory[2] = inventory[1];
                            inventory[1] = inventory[0];
                            inventory[0] = temp;

                        }

                        else if (power < 0){
                            temp = inventory[0];

                            inventory[0] = inventory[1];
                            inventory[1] = inventory[2];
                            inventory[2] = temp;
                        }

                        cycleCache = state;
                        cycleCount++;
                    }

                    if (cycleCount == numCycles){
                        //reachedGoal = true;
                        break;
                    }

                }

                caroselMotor.setPower(0);
            }
        }
         */

//
//        private void cycleToEmpty(){
//
//            int indexToCycle = getClosestEmpty();
//
//            //Left
//            if (indexToCycle == 1){
//                cycleCarosel(-1,1);
//            }
//
//            //Right
//            else if (indexToCycle == 2){
//                cycleCarosel(1,1);
//            }
//
//        }


        //April Tag Read
        public void updatePattern(String[] pattern){
            this.pattern = pattern;
        }


        //Pattern & Storage

        //Know Each COlor and Where they are (2d Array)
//        public String getStorageOne(){
//            return storage[0];
//        }
//
//        public String getStorageTwo(){
//            return storage[1];
//        }
//
//        public String getStorageThree(){
//            return storage[2];
//        }

        //Label Each of the 3 Ball Pods
        //Label Each of the 3 Possible Positions

        //Compare where each thing is and move accordingly

        //NEVERMIND, Keep Cycling Until Ball in right spto that easier

//        private void moveToStorageSpot(String color){
//
//            String[] currentInventory = storage;
//
//            int indexOfDesiredColor = -1;
//
//            for (int index = 0; index < currentInventory.length; index++){
//                if (currentInventory[index].equals(color)){
//                    indexOfDesiredColor = index;
//                    break;
//                }
//            }
//        }

        public void moveToStorageSpot(){

        }
    }
}