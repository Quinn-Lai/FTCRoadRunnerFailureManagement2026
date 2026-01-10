package org.firstinspires.ftc.teamcode.RobotV2.ClassData;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.Vector;

@Config
public class RobotConstantsV2 {

    /** Carosel Logic */
    private static final double netCaroselGearRatio = 1;
    private static final double analogVoltage = 3.3;
    private static final double conversionFactor = 355 / analogVoltage;
    public static final double encoderRes = conversionFactor * netCaroselGearRatio;
    public static final int caroselMultiplier = 100; //Manual Control
    public static final int CAROSEL_TOLERANCE = 1;
    public static int CAROSEL_INCREMENT = (int) encoderRes / 3; //Amount of bonus to add per unit of cycle
    public  static int CAROSEL_GLOBAL_INCREMENT = 0; //Position move relative to origin
    public static int CAROSEL_TOUCHPAD = 0;
    public static Double[] caroselPositions = new Double[]{0.07,0.45,0.82};

    /** Physics */
    public static double dragMultiplier = 2.55;
    public static int ANGLE_BONUS = 0;
    public static  double FAR_BALL_DISTANCE = 3.15;
    public static final double CLOSE_BALL_DISTANCE = 1.35; //1.35
    public static double HEIGHT_TO_AIM = 1.15;

    /** Transfer */
    public static final double TRANSFER_UP = 0.65;
    public static final double TRANSFER_DOWN = 0.87;
    public static final int AUTO_CYCLE_COOLDOWN = 0;

    /** COOLDOWNS */
    public static int FAILSAFE_SUBMODE_TIMER = 200;
    public static int COOLDOWN_SHOT = 200;
    public static int COOLDOWN_PRE_SHOT = 0;

    public static int FAILSAFE_SUBMODE_TIMER_AUTO = 100;
    public static int COOLDOWN_SHOT_AUTO = 300;
    public static int COOLDOWN_PRE_SHOT_AUTO = 100;

    /** Intake */
    public static final int INTAKE_ON = 1500;
    public static final int INTAKE_IN_EJECT = 1000;
    public static final int INTAKE_OFF = 0;
    public static final double RANGER_EQU_SLOPE = 32.50930976;
    public static final double RANGER_EQU_Y_INT = -2.6953;
    public static final double IN_TO_CM = 2.54;
    public static final int RANGER_DETECTION_MIN_THRESHOLD = 5;
    public static final int RANGER_DETECTION_MAX_THRESHOLD = 15;
    public static final int RANGER_DETECTION_CONFIRM_SHOT = 16;
    public static final int COLOR_SENSOR_DIST_THRESHOLD_FRONT = 2;
    public static final int COLOR_SENSOR_DIST_THRESHOLD_BACK = 4;

    public static double kPC = 0.0019;
    public static double kIC = 0.000021; //0.000005
    public static double kDC = 0.013; //0.13

    /** Modes */
    public static final String[] mainModes = new String[]{"auto","manual","humanIntake"};
    public static final String[] subModes = new String[]{"rapidFire","sortedFire","none"};
    public static final String[] subModeStages = new String[]{"cycle","transfer"};
    public static final int RAPID_FIRE_MAX_SHOTS = 3;

    /** Shooter */
    public static final int HUMAN_INTAKE_SPEED = 670;
    public static final int KILL_SHOOTER_SPEED = 0;
    public static final double FAR_TPS = 1600;
    public static final double MAX_HOOD_ANGLE = 64.79;
    public static final double MIN_HOOD_ANGLE = 51.3;
    public static final double MIN_HOOD_ANGLE_POS = 0.3;
    public static final double MAX_HOOD_ANGLE_POS = 0.75;
    public static final double SHOOTER_SPEED_THRESHOLD = 0.03; //Ready if shooter is within 3% of intended speed

    /** Limelight Parameters */
    public static final double LIMELIGHT_TURRET_DIFFERENCE = 0.09079;
    public static final double LIMELIGHT_AUTO_FAILSAFE = 3000;

    /** Indicator Colors */
    public static final double INDICATOR_GREEN = 0.5;
    public static final double INDICATOR_PURPLE = 0.722;
    public static final double INDICATOR_RED = 0.28;
    public static final double INDICATOR_ORANGE = 0.3;
    public static final double INDICATOR_YELLOW = 0.388;
    public static final double INDICATOR_BLUE = 0.611;


    /** Auto */
    public static final int AUTO_FAILSAFE_TIMER = 5000; //Pick up
    public static int AUTO_SLOW_SPEED = 20;
    public static final int AUTO_FAST_SPEED = 800;
    public static final int AUTO_SUPER_FAST_SPEED = 800;
    public static final double PINPOINT_HEADING_CLOSE_BLUE = 144.046; //54.046
    public static final double PINPOINT_HEADING_FAR_BLUE = 180;
    public static final double PINPOINT_HEADING_CLOSE_RED = 215.954; //305.954
    public static final double PINPOINT_HEADING_FAR_RED = 180;
    public static final double PINPOINT_TOLERENCE = 0.01;
    public static final Pose2d GATE_OPEN_POSITION_BLUE =  new Pose2d(11, -48, Math.toRadians(270));
    public static final Pose2d GATE_OPEN_POSITION_RED =  new Pose2d(11, 48, Math.toRadians(90));
    public static double INTAKE_TRAVEL = 45;


    /** TeleOp Road Runner */
    public static final Pose2d blueCorner = new Pose2d(61.5275,61.065,Math.toRadians(270));
    public static final Pose2d redCorner= new Pose2d(61.5275,-61.065,Math.toRadians(90));
    public static final Vector2d parkingBlue =  new Vector2d(36.7,32.5);
    public static final Vector2d parkingRed =  new Vector2d(36.7,-32.5);
    public static final double TRIGGER_TOLERENCE = 0.3;
    public static final double PARKING_TOLERENCE = 0.5;

    /** HSV Values */
    public static int MIDDLE_H = 170;
    public static double MIDDLE_S = 0.5;
    public static int MIDDLE_V = 1500;

    /** Misc */

    public static int ENDGAME_MARKER = 100;
    public static int FINAL_ENDGAME_MARKER = 110;


}
