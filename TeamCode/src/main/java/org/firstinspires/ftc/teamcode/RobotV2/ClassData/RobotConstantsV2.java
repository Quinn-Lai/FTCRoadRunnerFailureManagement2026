package org.firstinspires.ftc.teamcode.RobotV2.ClassData;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstantsV2 {
    /** Motors */
    private static final double netCaroselGearRatio = 1.45833333333;
    public static final double encoderRes = 384.5 * netCaroselGearRatio;
    public static int caroselMultiplier = 100;
    public static final double dragMultiplier = 1.2;
    public static final int MOTOR_TOLERENCE = 10;
    public static final int CAROSEL_SPEED = 1500;

    /** Carosel */
    public static Integer[] caroselPos = new Integer[]{0, (int) (encoderRes / 3), (int) (encoderRes * 2 / 3)};
    public static void updateCaroselPos(int bonus){
        caroselPos = new Integer[]{0 + bonus, (int) (encoderRes / 3) + bonus, (int) (encoderRes * 2 / 3) + bonus};
    }
    //public static final Integer[] caroselPos = new Integer[]{0, 207, 400};

    /** Intake */
    public static final double INTAKE_ON = 1800;
    public static final double INTAKE_OFF = 0;

    /** Transfer */
    public static final double TRANSFER_UP = 0.6;
    public static final double TRANSFER_DOWN = 0.9;
    public static final int COOLDOWN_SHOT = 200; //TODO Experiment with this value to decrease shot time

    /** Modes */
    public static final String[] mainModes = new String[]{"auto","manual","humanIntake"};
    public static final String[] subModes = new String[]{"rapidFire","sortedFire","none"};
    public static final String[] subModeStages = new String[]{"cycle","transfer"};
    public static final int RAPID_FIRE_MAX_SHOTS = 3;

    /** Fly Wheel Parameters */
    public static final int HUMAN_INTAKE_SPEED = 500;
    public static final int KILL_SHOOTER_SPEED = 0;
    public static final double FAR_BALL_DISTANCE = 3.0; //TODO Experiment with this value
    public static final double CLOSE_BALL_DISTANCE = 1.7901; //TODO Experiment with this value
    public static final double FAR_TPS = 1750; //TODO Experiment with This (or just use default calc)
    public static final int MAX_HOOD_ANGLE = 60; //TODOFFigure this out
    public static final int MIN_HOOD_ANGLE = 30;


    /** Hood Parameters */
    public static final int ANGLE_BONUS = 15;

    /** Limelight Parameters */
    public static final double LIMELIGHT_TURRET_DIFFERENCE = 0.16;

    /** Indicator Colors */
    public static final double INDICATOR_GREEN = 0.5;
    public static final double INDICATOR_PURPLE = 0.722;
    public static final double INDICATOR_RED = 0.28;
    public static final double INDICATOR_ORANGE = 0.3;
    public static final double INDICATOR_YELLOW = 0.388;
    public static final double INDICATOR_BLUE = 0.611;

    /** Indicator Logic */
    public static final double SHOOTER_MIN_SPEED_THRESHOLD = -0.01;
    public static final double SHOOTER_MAX_SPEED_THRESHOLD = 0.01;


}
