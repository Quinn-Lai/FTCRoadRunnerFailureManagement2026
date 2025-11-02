package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class TestLS extends LinearOpMode {

    private DigitalChannel magLS;

    @Override
    public void runOpMode(){

        magLS = hardwareMap.get(DigitalChannel.class,"magLS");

        magLS.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("Detected: ", magLS.getState());
            telemetry.update();
        }


    }

}
