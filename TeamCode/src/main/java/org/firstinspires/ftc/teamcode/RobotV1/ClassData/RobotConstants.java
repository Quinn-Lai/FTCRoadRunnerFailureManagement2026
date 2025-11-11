package org.firstinspires.ftc.teamcode.RobotV1.ClassData;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    public final double INTAKE_POWER = 1;
    public final double INTAKE_OFF = 0;
    private double encoderRes = 384.5; //435
    public final double longTPS = 1750;
    //public Integer[] caroselPos = new Integer[]{0,(int)encoderRes/3,(int)(encoderRes * 2/3)};
    public final int caroselIncrement = (int)encoderRes/3;
    public static double xDrag = 2.67;
    public static double yDrag = 2.67;
    public final double ELEVATOR_TOP = 0.3;
    public final double ELEVATOR_BOT = 1;
}
