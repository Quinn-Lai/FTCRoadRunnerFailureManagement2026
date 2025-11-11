package org.firstinspires.ftc.teamcode.RobotV2.ClassData;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstantsV2 {
    public static final double encoderRes = 384.5; //435
    public static final double longTPS = 1750;
    public static final Integer[] caroselPos = new Integer[]{0, (int) encoderRes / 3, (int) (encoderRes * 2 / 3)}; //TODO Add Turret Pos to array
    public static final double xDrag = 2.67;
    public static final double yDrag = 2.67;

    //Intake
    public static final double INTAKE_ON = 2000;
    public static final double INTAKE_OFF = 0;

    //Transfer
    public static final double TRANSFER_UP = 1; //TODO RANDOM NUMBER
    public static final double TRANSFER_DOWN = 0;
    public static final int COOLDOWN_SHOT = 200;


}
