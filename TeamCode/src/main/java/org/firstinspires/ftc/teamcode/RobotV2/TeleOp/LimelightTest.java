package org.firstinspires.ftc.teamcode.RobotV2.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class LimelightTest extends LinearOpMode {

    private Limelight3A limelight;
    //private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //imu = hardwareMap.get(IMU.class,"imu");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        waitForStart();

        limelight.start();

        while (opModeIsActive()) {
            //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            //limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
            }

            telemetry.update();
        }
    }
}