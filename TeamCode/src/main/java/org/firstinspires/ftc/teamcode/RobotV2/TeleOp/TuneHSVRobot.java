package org.firstinspires.ftc.teamcode.RobotV2.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotV2.ClassData.LimeLightVision;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RoadRunnerDataV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotConstantsV2;
import org.firstinspires.ftc.teamcode.RobotV2.ClassData.RobotDataV2;

import java.util.List;

@TeleOp
public class TuneHSVRobot extends OpMode {

    //Data Classes
    private RobotDataV2 robotData;         //Basic Robot Mechanics
    @Override
    public void init(){

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        robotData = new RobotDataV2(hardwareMap, telemetry);

    }

    @Override
    public void init_loop(){

    }

    //Runs Once on Start
    @Override
    public void start(){

    }

    //Loops after Start
    @Override
    public void loop(){



    }

    //Runs at End
    @Override
    public void stop(){
        robotData.getCarosel().killColorSensors();
    }
}