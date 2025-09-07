package org.firstinspires.ftc.teamcode.ClassData;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.opencv.ColorRange;


//Class to Organize each robot part and store the objects of each part of the robot
//Should be imported into RoadRunnerData to utilize each component
public class RobotData{

    //Drive Train
    private DcMotor LFmotor;
    private DcMotor RFmotor;
    private DcMotor LBmotor;
    private DcMotor RBmotor;

    //Sensors
    private IMU imu;

    //Hardware Map
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    //Time Utility
    private static ElapsedTime runtime;
    private ElapsedTime timer;

    //Booleans
    //private static boolean AUTO_RUN;
    private boolean robotCentric;
    private static boolean startedLeft;
    private boolean pendingColor;
    private boolean pendingPosition;


    //Driver
    private Gamepad mainDriver;
    //private Gamepad subDriver;
    private String tag;

    //OpenCV
    private boolean openCVEnabled;
    private static ColorRange startColor;

    //Constructor
    public RobotData(HardwareMap hardwareMap, Telemetry telemetry){

        //Get Hardware Map

        this.hardwareMap = hardwareMap;

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

        //Booleans
        //AUTO_RUN = false;
        robotCentric = true;
        openCVEnabled = false;
        pendingColor = true;
        pendingPosition = true;

        //OpenCV
        //startColor = ColorRange.RED; //Default in Case Auto Wasn't Run First

        //Turret
        CalculateAim turret = new CalculateAim();
        this.telemetry = telemetry;

        if (!robotCentric){
            imu = hardwareMap.get(IMU.class, "imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
            imu.initialize(parameters);
        }

        //For Delay Actions
        runtime = null;
        timer = new ElapsedTime();
    }

    //Accessor and Getter Methods

    public DcMotor getLFmotor(){
        return LFmotor;
    }

    public DcMotor getRFmotor() {
        return RFmotor;
    }

    public DcMotor getLBmotor() {
        return LBmotor;
    }

    public DcMotor getRBmotor() {
        return RBmotor;
    }

    public IMU getIMU() {
        return imu;
    }

//    public static boolean getAutoRun(){
//        return AUTO_RUN;
//    }
//
//    public static void setAutoRun(boolean run){
//        AUTO_RUN = run;
//    }
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
    public Gamepad getMainDriverGamepad() {
        return mainDriver;
    }

    public String getMainDriver(){
        return tag;
    }


    //Driver Select
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
    public void setMainDriver(Gamepad mainDriver, String tag) {

        //Only Changes Drivers if the person trying to swap isn't already the main driver
        //if (!(this.mainDriver.equals(mainDriver))){
            //this.subDriver = this.mainDriver;   //current main driver becomes P2
        this.mainDriver = mainDriver;       //new main driver becomes selected player
        this.tag = tag;
        this.mainDriver.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            //this.subDriver.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        //}
    }
    public void setMainDriver(Gamepad mainDriver, String tag, Gamepad optionalSubDriver) {
        //this.subDriver = optionalSubDriver;
        this.mainDriver = mainDriver;

        this.mainDriver.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        //this.subDriver.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        this.tag = tag;
    }

    //Utility Methods
    private void delayOrder(int delay) {
        timer.reset();
        while (timer.milliseconds() < delay) {
            omniDrive();
        }
    }
    public void displayTelemetryData(){
        telemetry.addData("Time: ", Math.floor(getRuntime()) + " Seconds");
        telemetry.addData("Main Driver: ", tag);
    }

    //Key Methods
    public void omniDrive(){

        double x = mainDriver.left_stick_x;
        double y = -mainDriver.left_stick_y;
        double z = mainDriver.right_stick_x;

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
    public void checkGameTimeRumble(){
        double currentRuntime = getRuntime();
        //telemetry.addLine("Time:" + currentRuntime + "Seconds");

        if (currentRuntime >= 90 && currentRuntime <= 92){
            mainDriver.rumble(2);
            //subDriver.rumble(2);
            mainDriver.setLedColor( 1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            //subDriver.setLedColor( 1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            telemetry.addLine("END GAME!");
        }

        else if (currentRuntime >= 110 && currentRuntime <= 120){
            mainDriver.rumble(2);
            //subDriver.rumble(2);
            double time = 120 - currentRuntime;
            telemetry.addLine(Math.floor(time) + " Seconds Left!");
        }
    }

//    private double convertServo(double angle, double lateralDegree,double servoAngle, double referenceDegree){
//        //x = desired angle in degrees
//        //y = servoAngle
//
//        double slope = (servoAngle)/(referenceDegree-lateralDegree);
//
//        return slope * (angle-lateralDegree); //Servo Degree converted from the angle
//    }


    private class PIDControl{
        private final double kPL = 0;

        private final double kIL = 0;
        private final double kDL = 0;

        private final double kPR = 0;

        private final double kIR = 0;
        private final double kDR = 0;


        private ElapsedTime PIDTimer = new ElapsedTime();

        private double integralSumL = 0;
        private double integralSumR = 0;
        private double lastErrorL = 0;
        private double lastErrorR = 0;

        public double PIDLeftSpinner(double current, double desired) {

            double currentTime = PIDTimer.seconds();

            double output = 0;

            double error = desired - current;

            integralSumL += error * currentTime;

            double derivate = (error - lastErrorL) / currentTime;

            lastErrorL = error;

            PIDTimer.reset();

            output = (error * kPL) + (derivate * kDL) + (integralSumL * kIL);

            return output;
        }

        public double PIDRightSpinner(double current, double desired) {

            double currentTime = PIDTimer.seconds();

            double output = 0;

            double error = desired - current;

            integralSumR += error * currentTime;

            double derivate = (error - lastErrorR) / currentTime;

            lastErrorR = error;

            PIDTimer.reset();

            output = (error * kPR) + (derivate * kDR) + (integralSumL * kIR);

            return output;
        }
    }

    private class CalculateAim extends PIDControl{

        //Motors
        private DcMotorEx leftSpinnerMotor;
        private DcMotorEx rightSpinnerMotor;

        //Physics Params
        private final double a = -9.8; //Acceleration m/s^2
        private double heightOfLauncher = 0.05; //Meters, need compensate for height of launcher
        private final double height = 1.05 - heightOfLauncher; //Meters
        private final double vY = Math.sqrt(2 * -a * height);
        private double vX = 0;

        //Motor Params
        private final double spinWheelRadius = 0.02; //Meters
        private final double w = 28.0; //Ticks

        private CalculateAim(){

            leftSpinnerMotor = (DcMotorEx)hardwareMap.get(DcMotor.class,"leftShooterMotor");
            rightSpinnerMotor = (DcMotorEx)hardwareMap.get(DcMotor.class,"rightShooterMotor");

            leftSpinnerMotor.setDirection(DcMotor.Direction.FORWARD);
            rightSpinnerMotor.setDirection(DcMotor.Direction.REVERSE); //Guess

            leftSpinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSpinnerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftSpinnerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightSpinnerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        }

        //Physics Calculations
        private double getYEquation(double time){
            return vY * time + 0.5 * a * Math.pow(time, 2);
        }

        private double getXEquation(double time){
            return vX * time;
        }

        private double getTimeExtrema(){
            return -vY/a; //2's cancel
        }

        private double getVy(){
            return vY;
        }

        private double getVx(double disp){
            return disp / getTimeExtrema();
        }

        private double getVelocityTotal(double disp){
            return Math.sqrt(Math.pow(getVy(),2) + Math.pow(getVx(disp),2));
        }

        private double getAngleTotal(){
            return Math.atan2(vY,vX);
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

        private void angleRobot(){

            double desiredAngle = getAngleTotal();

        }

        public void powerShooterMotors(double TPS){

            double leftCurrent = leftSpinnerMotor.getVelocity();
            double rightCurrent = rightSpinnerMotor.getVelocity();

            leftSpinnerMotor.setVelocity(PIDLeftSpinner(leftCurrent,TPS));
            rightSpinnerMotor.setVelocity(PIDRightSpinner(rightCurrent,TPS));
        }

        public void aimBall(double disp){

            angleRobot();

            double TPS = getTPS(disp);
            powerShooterMotors(TPS);

        }

    }

}