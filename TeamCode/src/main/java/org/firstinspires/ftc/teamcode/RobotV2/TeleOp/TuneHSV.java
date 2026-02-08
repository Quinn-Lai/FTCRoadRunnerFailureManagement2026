package org.firstinspires.ftc.teamcode.RobotV2.TeleOp;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp
public class TuneHSV extends LinearOpMode {

    private NormalizedColorSensor colorSensorFront;
    private NormalizedColorSensor colorSensorBack;
    private DistanceSensor frontDist;
    private DistanceSensor backDist;

    @Override
    public void runOpMode(){

        colorSensorFront = hardwareMap.get(NormalizedColorSensor.class,"colorSensorFront");
        colorSensorBack  = hardwareMap.get(NormalizedColorSensor.class,"colorSensorBack");
        frontDist = (DistanceSensor) colorSensorFront;
        backDist = (DistanceSensor) colorSensorBack;

        colorSensorFront.setGain(5);
        colorSensorBack.setGain(5);

        
        waitForStart();

        while (opModeIsActive()){
            float[] HSVFront = getHSVFront();
            telemetry.addLine(String.format("HSV Front: (%.5f, %.5f, %.5f)", HSVFront[0], HSVFront[1], HSVFront[2]));
            float[] HSVBack = getHSVBack();
            telemetry.addLine(String.format("HSV Back: (%.5f, %.5f, %.5f)", HSVBack[0], HSVBack[1], HSVBack[2]));

            telemetry.addData("Front Dist: ", frontDist.getDistance(DistanceUnit.CM));
            telemetry.addData("Back Dist: ", backDist.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

    }

    public float[] getHSVFront(){

        float[] hsv = new float[]{0F,0F,0F};

        NormalizedRGBA colors = colorSensorFront.getNormalizedColors();

        Color.RGBToHSV((int) (colors.red *255), (int) (colors.green *255), (int) (colors.blue * 255), hsv);
        return hsv;
    }
    public float[] getHSVBack(){

        float[] hsv = new float[]{0F,0F,0F};

        NormalizedRGBA colors = colorSensorBack.getNormalizedColors();

        Color.RGBToHSV((int)(colors.red *255), (int)(colors.green *255), (int)(colors.blue * 255), hsv);
        return hsv;
    }

}
