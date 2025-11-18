package org.firstinspires.ftc.teamcode.RobotV2.ClassData;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstantsV2 {
    /** Motors */
    public static final double encoderRes = 384.5; //435
    public static final double dragMultiplier = 1.2;
    public static final int MOTOR_TOLERENCE = 10;       //For Overrided IsBusy()

    /** Carosel */
    public static final Integer[] caroselPos = new Integer[]{0, (int) encoderRes / 3, (int) (encoderRes * 2 / 3)}; //TODO Add Turret Pos to array

    /** Intake */
    public static final double INTAKE_ON = 2000;
    public static final double INTAKE_OFF = 0;

    /** Transfer */
    public static final double TRANSFER_UP = 1; //TODO RANDOM NUMBER
    public static final double TRANSFER_DOWN = 0; //TODO RANDOM NUMBER
    public static final int COOLDOWN_SHOT = 200; //TODO Experiment with this value to decrease shot time

    /** Modes */
    public static final String[] mainModes = new String[]{"auto","manual","humanIntake"};
    public static final String[] subModes = new String[]{"rapidFire","sortedFire","none"};

    /** Fly Wheel Parameters */
    public static final int HUMAN_INTAKE_SPEED = 500;
    public static final int KILL_SHOOTER_SPEED = 0;
    public static double FAR_BALL_DISTANCE = 3.0; //TODO Experiment with this value
    public static final double FAR_TPS = 1750; //TODO Experiment with This (or just use default calc)

    /** Hood Parameters */
    public static final int ANGLE_BONUS = 15;


}
