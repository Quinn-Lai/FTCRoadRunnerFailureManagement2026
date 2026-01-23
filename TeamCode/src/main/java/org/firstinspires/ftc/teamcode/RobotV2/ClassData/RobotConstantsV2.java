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

    public static int FAILSAFE_SUBMODE_TIMER_AUTO = 200;
    public static int COOLDOWN_SHOT_AUTO = 200;
    public static int COOLDOWN_PRE_SHOT_AUTO = 0;

//    public static int FAILSAFE_SUBMODE_TIMER_AUTO = 100;
//    public static int COOLDOWN_SHOT_AUTO = 300;
//    public static int COOLDOWN_PRE_SHOT_AUTO = 100;

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
    public static final int COLOR_SENSOR_DIST_THRESHOLD_FRONT = 1;
    public static final double COLOR_SENSOR_DIST_THRESHOLD_BACK = 4;

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
    public static final int AUTO_FAILSAFE_TIMER = 2500; //Pick up
    public static int AUTO_SUPER_SLOW_SPEED = 15;
    public static int AUTO_SLOW_SPEED = 100;
    public static int AUTO_FAST_SPEED = 670;
    public static final int AUTO_SUPER_FAST_SPEED = 800;
    public static int AUTO_GATE_SPEED = 200;
    public static final double PINPOINT_HEADING_CLOSE_BLUE = 54.046; //144.046
    public static final double PINPOINT_HEADING_FAR_BLUE = 180;
    public static final double PINPOINT_HEADING_CLOSE_RED = 305.954; //215.954
    public static final double PINPOINT_HEADING_FAR_RED = 180;
    public static final double PINPOINT_TOLERENCE = 0.01;
    public static double ANGLE_TOLERENCE_AUTO = 2;

    /** Auto StartPos*/
    public static final Pose2d BLUE_SPAWN_FAR = new Pose2d(61.065+4, -13, Math.toRadians(RobotConstantsV2.PINPOINT_HEADING_FAR_BLUE));
    public static final Pose2d BLUE_SHOOT_FAR = new Pose2d(51+4, -13, Math.toRadians(203));
    public static final Pose2d BLUE_SPAWN_CLOSE = new Pose2d(-52, -48.5, Math.toRadians(RobotConstantsV2.PINPOINT_HEADING_CLOSE_BLUE));
    public static final Pose2d BLUE_SHOOT_CLOSE = new Pose2d(-15, -15, Math.toRadians(227)); //225
    public static final Pose2d OFF_LINE_BLUE_SHOOT_CLOSE = new Pose2d(-20, -15, Math.toRadians(230)); //225


    public static final Pose2d RED_SPAWN_FAR = new Pose2d(61.065+4, 13, Math.toRadians(RobotConstantsV2.PINPOINT_HEADING_FAR_RED));
    public static final Pose2d RED_SHOOT_FAR = new Pose2d(51+4, 13, Math.toRadians(157));
    public static final Pose2d RED_SPAWN_CLOSE = new Pose2d(-52, 48.5, Math.toRadians(RobotConstantsV2.PINPOINT_HEADING_CLOSE_RED));
    public static final Pose2d RED_SHOOT_CLOSE = new Pose2d(-15, 15, Math.toRadians(133)); //135
    public static final Pose2d OFF_LINE_RED_SHOOT_CLOSE = new Pose2d(-20, 15, Math.toRadians(130)); //135



    /** Auto Positional Increments*/

    public static double ANGLE_GATE = 90;
    public static double INTAKE_TRAVEL = 47;
    public static double PREP_TRAVEL = 24;
    public static double LINE_BOT_PREP_TRAVEL = 38;
    public static double LINE_MID_PREP_TRAVEL = 15;
    public static double LINE_TOP_PREP_TRAVEL = 9;
    public static double BELOW_TOP_INCREMENT = 12; //8


    /** Auto Blue Pos*/
    public final static Pose2d LINE_BOT_PREP_BLUE = new Pose2d(RobotConstantsV2.LINE_BOT_PREP_TRAVEL, -RobotConstantsV2.PREP_TRAVEL, Math.toRadians(270));
    public final static Pose2d LINE_BOT_COLLECT_BLUE = new Pose2d(RobotConstantsV2.LINE_BOT_PREP_TRAVEL, -RobotConstantsV2.INTAKE_TRAVEL - RobotConstantsV2.BELOW_TOP_INCREMENT, Math.toRadians(270));
    public final static Pose2d LINE_MID_PREP_BLUE = new Pose2d(RobotConstantsV2.LINE_MID_PREP_TRAVEL, -RobotConstantsV2.PREP_TRAVEL, Math.toRadians(270));
    public final static Pose2d LINE_MID_COLLECT_BLUE = new Pose2d(RobotConstantsV2.LINE_MID_PREP_TRAVEL, -RobotConstantsV2.INTAKE_TRAVEL - RobotConstantsV2.BELOW_TOP_INCREMENT, Math.toRadians(270));
    public final static Pose2d LINE_TOP_PREP_BLUE = new Pose2d(-RobotConstantsV2.LINE_TOP_PREP_TRAVEL, -RobotConstantsV2.PREP_TRAVEL, Math.toRadians(270));
    public final static Pose2d LINE_TOP_COLLECT_BLUE = new Pose2d(-RobotConstantsV2.LINE_TOP_PREP_TRAVEL, -RobotConstantsV2.INTAKE_TRAVEL-2, Math.toRadians(270));
    public final static Pose2d GATE_LINE_MID_PREP_BLUE = new Pose2d(11, -RobotConstantsV2.PREP_TRAVEL, Math.toRadians(270));
    public final static Pose2d GATE_OPEN_POSITION_BLUE =  new Pose2d(11, -52, Math.toRadians(270));
    public final static Pose2d COLLECT_GATE_CLOSE_BLUE = new Pose2d(0,0,0);
    public final static Pose2d COLLECT_GATE_FAR_BLUE = new Pose2d(0,0,0);
    public final static Pose2d SLAM_GATE_BLUE = new Pose2d(1,-53,Math.toRadians(270));

    /** Auto Red Pos*/
    public final static Pose2d LINE_BOT_PREP_RED = new Pose2d(RobotConstantsV2.LINE_BOT_PREP_TRAVEL, RobotConstantsV2.PREP_TRAVEL, Math.toRadians(90));
    public final static Pose2d LINE_BOT_COLLECT_RED = new Pose2d(RobotConstantsV2.LINE_BOT_PREP_TRAVEL, RobotConstantsV2.INTAKE_TRAVEL+ RobotConstantsV2.BELOW_TOP_INCREMENT, Math.toRadians(90));
    public final static Pose2d LINE_MID_PREP_RED = new Pose2d(RobotConstantsV2.LINE_MID_PREP_TRAVEL, RobotConstantsV2.PREP_TRAVEL, Math.toRadians(90));
    public final static Pose2d LINE_MID_COLLECT_RED = new Pose2d(RobotConstantsV2.LINE_MID_PREP_TRAVEL, RobotConstantsV2.INTAKE_TRAVEL + RobotConstantsV2.BELOW_TOP_INCREMENT, Math.toRadians(90));
    public final static Pose2d LINE_TOP_PREP_RED = new Pose2d(-RobotConstantsV2.LINE_TOP_PREP_TRAVEL, RobotConstantsV2.PREP_TRAVEL, Math.toRadians(90));
    public final static Pose2d LINE_TOP_COLLECT_RED = new Pose2d(-RobotConstantsV2.LINE_TOP_PREP_TRAVEL, RobotConstantsV2.INTAKE_TRAVEL+2, Math.toRadians(90));
    public final static Pose2d GATE_LINE_MID_PREP_RED = new Pose2d(11, RobotConstantsV2.PREP_TRAVEL, Math.toRadians(90));
    public final static Pose2d GATE_OPEN_POSITION_RED =  new Pose2d(11, 52, Math.toRadians(90));
    public final static Pose2d COLLECT_GATE_CLOSE_RED = new Pose2d(0,0,0);
    public final static Pose2d COLLECT_GATE_FAR_RED = new Pose2d(0,0,0);
    public final static Pose2d SLAM_GATE_RED = new Pose2d(-1,53,Math.toRadians(90));

    /** Auto Speed Control */

    public static int MAX_ACCEL_SPEED = 150;
    public static int MIN_ACCEL_SPEED = -30; //-30
    public static int MAX_VEL_SPEED = 100;
    public static int MAX_ACCEL_DEFAULT = 50;
    public static int MIN_ACCEL_DEFAULT = -30;
    public static int MAX_VEL_DEFAULT = 50;

    /** TeleOp Road Runner */
    public static final Pose2d blueCorner = new Pose2d(61.5275,61.065,Math.toRadians(270));
    public static final Pose2d redCorner= new Pose2d(61.5275,-61.065,Math.toRadians(90));
    public static final Pose2d parkingBlue =  new Pose2d(36.7,32.5,Math.toRadians(270));
    public static final Pose2d parkingRed =  new Pose2d(36.7,-32.5, Math.toRadians(90));
    public static final double TRIGGER_TOLERENCE = 0.3;
    public static final double PARKING_TOLERENCE = 1;
    public static final double PARKING_SPEED = 50;

    /** HSV Values */
    public static int MIDDLE_H = 165; //170
    public static double MIN_S = 0.44f;
    public static double MIN_V = .18f;
    public static int SENSOR_GAIN = 5;

    /** Misc */

    public static int ENDGAME_MARKER = 100;
    public static int FINAL_ENDGAME_MARKER = 110;


}
