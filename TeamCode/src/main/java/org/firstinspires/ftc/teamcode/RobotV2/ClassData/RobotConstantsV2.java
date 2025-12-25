package org.firstinspires.ftc.teamcode.RobotV2.ClassData;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class RobotConstantsV2 {

    /** Carosel Logic */
    private static final double netCaroselGearRatio = 1;
    private static final double analogVoltage = 3.3;
    private static final double conversionFactor = 360 / analogVoltage;
    public static final double encoderRes = conversionFactor * netCaroselGearRatio;
    public static int caroselMultiplier = 100; //Manual Control
    public static final int CAROSEL_TOLERANCE = 5;
    public static int CAROSEL_INCREMENT = (int) encoderRes / 3; //Amount of bonus to add per unit of cycle
    public  static int CAROSEL_GLOBAL_INCREMENT = 0; //Position move relative to origin
    public static int CAROSEL_TOUCHPAD = 0;

    /** Physics */
    public static final double dragMultiplier = 2.1;
    public static final int ANGLE_BONUS = 21;

    /** Transfer */
    public static final int TRANSFER_COOLDOWN = 300; //TODO honestly forgot what this was for
    public static final int COOLDOWN_SHOT = 300; //Transfer Shot
    public static final double TRANSFER_UP = 0.6;
    public static final double TRANSFER_DOWN = 0.87;
    public static final int COOLDOWN_INTAKE = 500;


    /** Intake */
    public static final int INTAKE_ON = 1500;
    public static final int INTAKE_IN_EJECT = 1000;
    public static final int INTAKE_OFF = 0;
    public static final double RANGER_EQU_SLOPE = 32.50930976;
    public static final double RANGER_EQU_Y_INT = -2.6953;
    public static final double IN_TO_CM = 2.54;
    public static final int RANGER_DETECTION_MIN_THRESHOLD = 3;
    public static final int RANGER_DETECTION_MAX_THRESHOLD = 5; //TODO tune these


    /** Modes */
    public static final String[] mainModes = new String[]{"auto","manual","humanIntake"};
    public static final String[] subModes = new String[]{"rapidFire","sortedFire","none"};
    public static final String[] subModeStages = new String[]{"cycle","transfer"};
    public static final int FAILSAFE_SUBMODE_TIMER = 1000;
    public static final int RAPID_FIRE_MAX_SHOTS = 3;

    /** Shooter */
    public static final int HUMAN_INTAKE_SPEED = 500;
    public static final int KILL_SHOOTER_SPEED = 0;
    public static final double FAR_BALL_DISTANCE = 3.1;
    public static final double CLOSE_BALL_DISTANCE = 1.9;
    public static final double FAR_TPS = 1600;
    public static final double MAX_HOOD_ANGLE = 65.0;
    public static final double MIN_HOOD_ANGLE = 51.3;
    public static final double SHOOTER_SPEED_THRESHOLD = 0.03; //Ready if shooter is within 3% of intended speed

    /** Limelight Parameters */
    public static final double LIMELIGHT_TURRET_DIFFERENCE = 0.19079;

    /** Indicator Colors */
    public static final double INDICATOR_GREEN = 0.5;
    public static final double INDICATOR_PURPLE = 0.722;
    public static final double INDICATOR_RED = 0.28;
    public static final double INDICATOR_ORANGE = 0.3;
    public static final double INDICATOR_YELLOW = 0.388;
    public static final double INDICATOR_BLUE = 0.611;


    /** Auto */
    public static final int AUTO_FAILSAFE_TIMER = 11000; //Pick up
    public static final int AUTO_SLOW_SPEED = 2;
    public static final int AUTO_FAST_SPEED = 500;
    public static final int AUTO_SUPER_FAST_SPEED = 800;


    /** TeleOp Road Runner */
    public static final Pose2d blueCorner = new Pose2d(61.065,61.5275,Math.toRadians(180));
    public static final Pose2d redCorner= new Pose2d(61.065,-61.5275,Math.toRadians(180));
    public static final Vector2d parkingBlue =  new Vector2d(36.7,32.5);
    public static final Vector2d parkingRed =  new Vector2d(36.7,-32.5);

    /** HSV Values */
    public static int MIDDLE_H = 200;
    public static double MIDDLE_S = 0.55;


}
