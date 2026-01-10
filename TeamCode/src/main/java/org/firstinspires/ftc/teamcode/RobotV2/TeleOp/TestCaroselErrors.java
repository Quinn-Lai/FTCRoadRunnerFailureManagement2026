package org.firstinspires.ftc.teamcode.RobotV2.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotConstantsV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotDataV2;

@TeleOp
public class TestCaroselErrors extends LinearOpMode {

    private RobotDataV2 robotData;

    @Override
    public void runOpMode(){

        robotData = new RobotDataV2(hardwareMap,telemetry);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.squareWasPressed()){
                robotData.getCarosel().cycleCaroselManual();
            }

            else if (gamepad1.triangle){
                telemetry.addLine("POOP");
            }

            telemetry.addData("Carosel In Place Status: ", robotData.getCarosel().isCaroselInPlace());

            telemetry.update();
        }

    }

}
