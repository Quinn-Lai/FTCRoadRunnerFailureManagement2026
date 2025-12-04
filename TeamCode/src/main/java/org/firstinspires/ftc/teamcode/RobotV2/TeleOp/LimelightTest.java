package org.firstinspires.ftc.teamcode.RobotV2.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.LimeLightVision;

@Disabled
@TeleOp
public class LimelightTest extends LinearOpMode {

    private LimeLightVision limelight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = new LimeLightVision(hardwareMap, telemetry, "blue");

        waitForStart();

        limelight.initLimeLight();

        while (opModeIsActive()) {
            //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            //limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            if (limelight.getResults() != null && limelight.getResults().isValid()) {
                telemetry.addData("tx", limelight.getTx());
                telemetry.addData("ty", limelight.getTy()); //TODO test multiple april tags
                telemetry.addData("ty deg", limelight.getTyFidDeg());
                telemetry.addData("Displacement: ", limelight.getDisp());
            }

            telemetry.update();
        }
    }
}