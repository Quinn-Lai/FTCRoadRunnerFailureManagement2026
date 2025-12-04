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
                robotData.getCarosel().getCaroselMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            telemetry.addData("Carosel Target Position: ", robotData.getCarosel().getCaroselMotor().getTargetPosition());
            telemetry.addData("Carosel Current Position: ", robotData.getCarosel().getCaroselMotor().getCurrentPosition());
            telemetry.addData("Error: ", robotData.getCarosel().getCaroselMotor().getTargetPosition() - robotData.getCarosel().getCaroselMotor().getCurrentPosition());
            telemetry.addData("Carosel In Place Status: ", robotData.getCarosel().isCaroselInPlace());
            telemetry.addData("Motor Tolorence: ", RobotConstantsV2.MOTOR_TOLERENCE);

            telemetry.update();
        }

    }

}
