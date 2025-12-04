package org.firstinspires.ftc.teamcode.RobotV2.ClassData;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class RobotConstantsV2 {
    /** Motors */
    private static final double netCaroselGearRatio = 0.5;
    public static final double encoderRes = 1425.1 * netCaroselGearRatio;
    public static int caroselMultiplier = 100;
    public static final double dragMultiplier = 2.1;
    public static final int MOTOR_TOLERENCE = 15;
    public static final int CAROSEL_SPEED = 1500;

    public static final int TRANSFER_COOLDOWN = 200;
    public static final int COOLDOWN_SHOT = 300; //TODO Experiment with this value to decrease shot time

    /** Carosel */
    public static Integer[] caroselPos = new Integer[]{0, (int) (encoderRes / 3), (int) (encoderRes * 2 / 3)};
    public static void updateCaroselPos(int bonus){
        caroselPos = new Integer[]{0 + bonus, (int) (encoderRes / 3) + bonus, (int) (encoderRes * 2 / 3) + bonus};
    }
    //public static final Integer[] caroselPos = new Integer[]{0, 207, 400};

    /** Intake */
    public static final int INTAKE_ON = 1500;
    public static final int INTAKE_IN_EJECT = 1000;
    public static final int INTAKE_OFF = 0;

    /** Transfer */
    public static final double TRANSFER_UP = 0.6;
    public static final double TRANSFER_DOWN = 0.87;
    public static final int COOLDOWN_INTAKE = 200;

    /** Modes */
    public static final String[] mainModes = new String[]{"auto","manual","humanIntake"};
    public static final String[] subModes = new String[]{"rapidFire","sortedFire","none"};
    public static final String[] subModeStages = new String[]{"cycle","transfer"};
    public static final int FAILSAFE_SUBMODE_TIMER = 1000;
    public static final int RAPID_FIRE_MAX_SHOTS = 3;

    /** Fly Wheel Parameters */
    public static final int HUMAN_INTAKE_SPEED = 500;
    public static final int KILL_SHOOTER_SPEED = 0;
    public static final double FAR_BALL_DISTANCE = 3.0; //TODO Experiment with this value
    public static final double CLOSE_BALL_DISTANCE = 1.8; //1.7901 //TODO Experiment with this value
    public static final double FAR_TPS = 1600; //TODO Experiment with This (or just use default calc)
    public static final int MAX_HOOD_ANGLE = 65;
    public static final int MIN_HOOD_ANGLE = 47;


    /** Hood Parameters */
    public static final int ANGLE_BONUS = 21;

    /** Limelight Parameters */
    public static final double LIMELIGHT_TURRET_DIFFERENCE = 0.19079;

    /** Indicator Colors */
    public static final double INDICATOR_GREEN = 0.5;
    public static final double INDICATOR_PURPLE = 0.722;
    public static final double INDICATOR_RED = 0.28;
    public static final double INDICATOR_ORANGE = 0.3;
    public static final double INDICATOR_YELLOW = 0.388;
    public static final double INDICATOR_BLUE = 0.611;

    /** Indicator Logic */
    public static final double SHOOTER_SPEED_THRESHOLD = 0.03;

    /** Auto */

    public static final int AUTO_FAILSAFE_TIMER = 5000;
    public static final int AUTO_SLOW_SPEED = 5;

    public static final Pose2d blueCorner = new Pose2d(61.065,61.5275,Math.toRadians(180));
    public static final Pose2d redCorner= new Pose2d(61.065,-61.5275,Math.toRadians(180));
    public static final Vector2d parkingBlue =  new Vector2d(36.7,32.5);
    public static final Vector2d parkingRed =  new Vector2d(36.7,-32.5);

}
