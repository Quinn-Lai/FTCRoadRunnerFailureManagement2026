package org.firstinspires.ftc.teamcode.RobotV2.ClassData;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

@Config
public class RobotConstantsV2 {

    /** Cooldowns & Parameters Panel */

    /** Intaking */
    public static int CAROSEL_DETECTED_ARTIFACT_DELAY = 500;
    public static int CAROSEL_DETECTED_ARTIFACT_DELAY_TELE = 500;
    public static int CAROSEL_DETECTED_ARTIFACT_DELAY_AUTO = 500;
    public static int CAROSEL_DETECTED_ARTIFACT_DELAY_LAST_SLOT = 670;

    /** Analog Tolerence */

    public static int CAROSEL_TOLERANCE = 0;
    public static int CAROSEL_TOLERENCE_INTAKE = 7; //10
    public static int CAROSEL_TOLERANCE_TELE = 0; //15
    public static int CAROSEL_TOLERANCE_AUTO = 10; //10

    /** Submode Timings */
    public static int FAILSAFE_SUBMODE_TIMER = 100;
    public static int FAILSAFE_SUBMODE_TIMER_LONG = 450;
    public static int COOLDOWN_SHOT_UP = 85; //75
    public static int COOLDOWN_SHOT_DOWN = 85;

    /** Failsafe For End Of Action */
    public static int AUTO_FAILSAFE_TIMER = 3500; //Pick up
    public static int AUTO_FAILSAFE_TIMER_CLOSE = 3500; //Pick up
    public static int AUTO_FAILSAFE_TIMER_FAR = 5500; //Pick up


    /** Carosel Logic */
    private static final double netCaroselGearRatio = 2;
    private static final double analogVoltage = 3.3;
    private static final double conversionFactor = 355 / analogVoltage;
    public static final double encoderRes = conversionFactor * netCaroselGearRatio;

    public static final int CAROSEL_INCREMENT = (int) encoderRes / 3; //Amount of bonus to add per unit of cycle
    public static Double[] caroselPositions = new Double[]{0.0,0.45/2 - 0.07/2,0.82/2 - 0.07/2};


    /** Physics */
    public static double dragMultiplier = 2.55;
    public static double VERTICAL_MULTIPLIER = 1;
    public static int ANGLE_BONUS = 0;
    public static double FAR_BALL_DISTANCE = 3.11;
    public static double CLOSE_BALL_DISTANCE = 1.7; //1.53
    public static double OFF_LINE_BALL_DISTANCE = 1.53; //1.53
    public static double HEIGHT_TO_AIM = 1.8;
    public static double MAX_TURRET_TPS = 1600;
    public static double DIST_CHANGE_THRESHOLD = 2.8;

    /** Transfer */
    public static double TRANSFER_UP = 0.6; //0.6
    public static double TRANSFER_DOWN = 0.85;
    public static final int AUTO_CYCLE_COOLDOWN = 0;


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
    public static double COLOR_SENSOR_DIST_THRESHOLD_FRONT = 0.67;
    public static double COLOR_SENSOR_DIST_THRESHOLD_BACK = 2;

    /** PID Constants*/

    public static double kP = 0.008;
    public static double kI = 0.00000015;
    public static double kD = 0.0000015;
    public static double kF = 0.00048;
    public static double kPHeading = 0.02;
    public static double kIHeading = 0.00000001;//0.000021
    public static double kDHeading = 0.0000000001;//0.001

    /** Modes */
    public static final String[] mainModes = new String[]{"auto","manual","humanIntake"};
    public static final String[] subModes = new String[]{"rapidFire","sortedFire","none"};
    public static final String[] subModeStages = new String[]{"cycle","transfer"};
    public static final int RAPID_FIRE_MAX_SHOTS = 3;

    /** Shooter */
    public static final int HUMAN_INTAKE_SPEED = 670;
    public static final int KILL_SHOOTER_SPEED = 0;
    public static final double FAR_TPS = 1600;
    public static final double MAX_HOOD_ANGLE = 48.3;
    public static final double MIN_HOOD_ANGLE = 27.3;
    public static final double MIN_HOOD_ANGLE_POS = 0.6;
    public static final double MAX_HOOD_ANGLE_POS = 0.75;
    public static final double SHOOTER_SPEED_THRESHOLD = 0.03; //Ready if shooter is within 3% of intended speed

    /** Limelight Parameters */
    public static double LIMELIGHT_TURRET_DIFFERENCE = 0.19079;
    public static final double LIMELIGHT_AUTO_FAILSAFE = 3000;

    /** Indicator Colors */
    public static final double INDICATOR_GREEN = 0.5;
    public static final double INDICATOR_PURPLE = 0.722;
    public static final double INDICATOR_RED = 0.28;
    public static final double INDICATOR_ORANGE = 0.3;
    public static final double INDICATOR_YELLOW = 0.388;
    public static final double INDICATOR_BLUE = 0.611;


    /** Auto */
    public static final double PINPOINT_HEADING_CLOSE_BLUE = 54.046; //144.046
    public static final double PINPOINT_HEADING_FAR_BLUE = 180;
    public static final double PINPOINT_HEADING_CLOSE_RED = 305.954; //215.954
    public static final double PINPOINT_HEADING_FAR_RED = 180;
    public static final double PINPOINT_TOLERENCE = 0.01;
    public static double ANGLE_TOLERENCE_AUTO = 5;
    public static double FAR_CORNER_POS_TOLERENCE = 3;

    /** Auto StartPos*/
    public static final Pose2d BLUE_SPAWN_FAR = new Pose2d(61.065+3, -13, Math.toRadians(RobotConstantsV2.PINPOINT_HEADING_FAR_BLUE));
    public static final Pose2d BLUE_SHOOT_FAR = new Pose2d(51, -13, Math.toRadians(203));
    public static final Pose2d BLUE_SPAWN_CLOSE = new Pose2d(-52, -48.5, Math.toRadians(RobotConstantsV2.PINPOINT_HEADING_CLOSE_BLUE));
    public static final Pose2d BLUE_SHOOT_CLOSE = new Pose2d(-9, -12, Math.toRadians(220)); //225
    public static final Pose2d OFF_LINE_BLUE_SHOOT_CLOSE = new Pose2d(-30, -15, Math.toRadians(238)); //225

    public static final Pose2d RED_SPAWN_FAR = new Pose2d(61.065+3, 13, Math.toRadians(RobotConstantsV2.PINPOINT_HEADING_FAR_RED));
    public static final Pose2d RED_SHOOT_FAR = new Pose2d(51, 13, Math.toRadians(157));
    public static final Pose2d RED_SPAWN_CLOSE = new Pose2d(-52, 48.5, Math.toRadians(RobotConstantsV2.PINPOINT_HEADING_CLOSE_RED));
    public static final Pose2d RED_SHOOT_CLOSE = new Pose2d(-9, 12, Math.toRadians(140)); //135
    public static final Pose2d OFF_LINE_RED_SHOOT_CLOSE = new Pose2d(-30, 15, Math.toRadians(122)); //135


    public static final double PINPOINT_HEADING_CLOSE_BLUE_BACKWARD = 54.046 + 180;
    public static final double PINPOINT_HEADING_CLOSE_RED_BACKWARD = 305.954 - 180;
    public static final Pose2d BLUE_SPAWN_CLOSE_BACKWARD = new Pose2d(-52, -48.5, Math.toRadians(RobotConstantsV2.PINPOINT_HEADING_CLOSE_BLUE_BACKWARD));
    public static final Pose2d RED_SPAWN_CLOSE_BACKWARD = new Pose2d(-52, 48.5, Math.toRadians(RobotConstantsV2.PINPOINT_HEADING_CLOSE_RED_BACKWARD));

    /** Auto Blue Pos*/
    public final static Pose2d LINE_BOT_PREP_BLUE = new Pose2d(38, -24, Math.toRadians(270));
    public final static Pose2d LINE_BOT_COLLECT_BLUE = new Pose2d(38, -43, Math.toRadians(270)); //-49
    public final static Pose2d LINE_MID_PREP_BLUE = new Pose2d(15, -24, Math.toRadians(270));
    public final static Pose2d LINE_MID_COLLECT_BLUE = new Pose2d(15, -43, Math.toRadians(270)); //-59
    public final static Pose2d LINE_TOP_PREP_BLUE = new Pose2d(-9, -24, Math.toRadians(270));
    public final static Pose2d LINE_TOP_COLLECT_BLUE = new Pose2d(-9, -43, Math.toRadians(270));
    public final static Pose2d GATE_MIDDLE_LINE_OPEN_BLUE =  new Pose2d(8, -52, Math.toRadians(270));
    public final static Pose2d GATE_INTAKE_BLUE = new Pose2d(13,-55,Math.toRadians(240));
    public final static Pose2d SLAM_GATE_BLUE = new Pose2d(3,-53,Math.toRadians(270));
    public final static Pose2d PARK_GATE_BLUE = new Pose2d(3, -43, Math.toRadians(0));
    public final static Pose2d CORNER_BLUE = new Pose2d(70, -60, Math.toRadians(270)); //315
    public final static Pose2d CORNER_RED = new Pose2d(70, 60, Math.toRadians(90)); //45
    public final static Pose2d CORNER_BLUE_SHIMMY = new Pose2d(55, -60, Math.toRadians(270));
    public final static Pose2d CORNER_RED_SHIMMY = new Pose2d(55, 60, Math.toRadians(90));


    /** Auto Red Pos*/
    public final static Pose2d LINE_BOT_PREP_RED = new Pose2d(38, 24, Math.toRadians(90));
    public final static Pose2d LINE_BOT_COLLECT_RED = new Pose2d(38, 43, Math.toRadians(90));
    public final static Pose2d LINE_MID_PREP_RED = new Pose2d(15, 24, Math.toRadians(90));
    public final static Pose2d LINE_MID_COLLECT_RED = new Pose2d(15, 43, Math.toRadians(90));
    public final static Pose2d LINE_TOP_PREP_RED = new Pose2d(-9, 24, Math.toRadians(90));
    public final static Pose2d LINE_TOP_COLLECT_RED = new Pose2d(-9, 43, Math.toRadians(90));
    public final static Pose2d GATE_MIDDLE_LINE_OPEN_RED =  new Pose2d(8, 52, Math.toRadians(90));
    public final static Pose2d GATE_INTAKE_RED = new Pose2d(13,55,Math.toRadians(120));
    public final static Pose2d SLAM_GATE_RED = new Pose2d(3,53,Math.toRadians(90));
    public final static Pose2d PARK_GATE_RED = new Pose2d(3, 43, Math.toRadians(0));

    /** Auto Speed Control */
    public static int AUTO_SUPER_SLOW_SPEED = 10;
    public static int AUTO_SLOW_SPEED = 100;
    public static int AUTO_FAST_SPEED = 670;
    public static int AUTO_GATE_SPEED = 200;
    public static int MAX_ACCEL_SPEED = 250;
    public static int MIN_ACCEL_SPEED = -32; //-30
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
    public static final double PARKING_SPEED = 67;
    public static final Double[] GLOBAL_GOAL_POS_RED = new Double[]{-70.0,70.0};
    public static final Double[] GLOBAL_GOAL_POS_BLUE = new Double[]{-70.0,-70.0};
    public static final Double[] GLOBAL_GOAL_AIM_RED = new Double[]{-58.0,58.0}; //-59, 53
    public static final Double[] GLOBAL_GOAL_AIM_BLUE = new Double[]{-58.0,-58.0}; //-59, -53
    public static Pose2d LAST_ROBOT_POS = new Pose2d(0,0,0);


    /** HSV Values */
    public static int MIDDLE_H = 165; //170
    public static double MIN_S = 0.44f;
    public static double MIN_V = .035f; //0.067
    public static int SENSOR_GAIN = 7;
    public static String FAILSAFE_INTAKE = "Purple";

    /** Misc */

    public static int ENDGAME_MARKER = 100;
    public static int FINAL_ENDGAME_MARKER = 110;


}
