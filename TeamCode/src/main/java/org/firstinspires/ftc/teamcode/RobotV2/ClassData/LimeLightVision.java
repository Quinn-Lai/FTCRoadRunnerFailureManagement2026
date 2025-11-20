package org.firstinspires.ftc.teamcode.RobotV2.ClassData;

import android.util.Size;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.jvm.Gen;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class LimeLightVision {

    /** Hardware */
    private Limelight3A limelight;
    private GoBildaPinpointDriver imu;

    /** Variables */
    private double atHeight; //Meters
    private String[] motifCode;
    private String alliance;
    private Telemetry telemetry;

    /** Limelight Variables */
    private double lensHeight;
    private double levelLensAtHeight;


    //----------------------------------------
    /** Constructor */
    public LimeLightVision(HardwareMap hardwareMap, Telemetry telemetry, String alliance){

        /** Hardware Init */
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);

        /** Pinpoint Init */
        imu = hardwareMap.get(GoBildaPinpointDriver.class, "imu");

        GoBildaPinpointDriver.Register[] defaultRegisters = {
                GoBildaPinpointDriver.Register.DEVICE_STATUS,
                GoBildaPinpointDriver.Register.LOOP_TIME,
                GoBildaPinpointDriver.Register.X_ENCODER_VALUE,
                GoBildaPinpointDriver.Register.Y_ENCODER_VALUE,
                GoBildaPinpointDriver.Register.X_POSITION,
                GoBildaPinpointDriver.Register.Y_POSITION,
                GoBildaPinpointDriver.Register.H_ORIENTATION,
                GoBildaPinpointDriver.Register.X_VELOCITY,
                GoBildaPinpointDriver.Register.Y_VELOCITY,
                GoBildaPinpointDriver.Register.H_VELOCITY,
        };

        GoBildaPinpointDriver.Register[] onlyPosition = {
                GoBildaPinpointDriver.Register.DEVICE_STATUS,
                GoBildaPinpointDriver.Register.X_POSITION,
                GoBildaPinpointDriver.Register.Y_POSITION,
                GoBildaPinpointDriver.Register.H_ORIENTATION,
        };

        imu.setBulkReadScope(defaultRegisters);
        imu.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);

//        imu.setOffsets(-84.0, -168.0, DistanceUnit.MM); //TODO Tune offsets, Might not need tho
//        imu.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        imu.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        imu.resetPosAndIMU();

        /** Alliance */
        this.alliance = alliance;
        if (alliance.equals("blue")) limelight.pipelineSwitch(0);
        else limelight.pipelineSwitch(1);

        /** Variable Init */
        this.atHeight = 0.756015; //approximation 0.756015
        motifCode = null;
        this.telemetry = telemetry;

        lensHeight = 0.3; //TODO Tune this
        levelLensAtHeight = atHeight - lensHeight;
    }

    //----------------------------------------

    /** Limelight */
    public void initLimeLight(){
        limelight.start();
    }
    public void killLimeLight(){
        limelight.close();
    }
    public LLResult getResults(){
        return limelight.getLatestResult();
    }
    public double getTx(){
        return getResults().getTx();
    }
    public double getTy(){ //TODO Might cause error if see two april tags (could blacklist oblisk after it sees it or just check id)
        return getResults().getTy();
    }

    /** IMU Pinpoint */
    public void recalibrateIMUStatus(){
        imu.recalibrateIMU();;
    }
    public double getYaw(){ //TODO might not be the same as april tag library pipline
        return imu.getHeading(AngleUnit.DEGREES);
    }

    /** Data */
    private double getTyDeg(){
        return getTy() * (Math.PI/180);
    }
    public double getTyFidDeg(){

        if (getResults().isValid()){
            List<LLResultTypes.FiducialResult> fiducials = getResults().getFiducialResults();

            for (LLResultTypes.FiducialResult f : fiducials) {
                if (f.getFiducialId() != 21 && f.getFiducialId() != 22 && f.getFiducialId() != 23){
                    //TODO Weird way out, pipline doesn't search for it anyways, so just check what's not id
                    return f.getTargetYDegrees();
                }
            }
        }

        return 0;

    }
    public double getDisp(){
        if (getResults().isValid()){
            return (levelLensAtHeight / Math.tan(getTyFidDeg())) + RobotConstantsV2.LIMELIGHT_TURRET_DIFFERENCE; //Another triangle here upsidedown
        }
        return 0;
    } //TODO need to account for height difference and displacement (prob jsut add center of lens to launch zone)

    /** Last Updates */
    public void telemetryLimeLight(){
        if (getResults() != null && getResults().isValid()) {
            telemetry.addData("tX", getTx());
            telemetry.addData("tY", getTy());
        }
    }
    public void updateOrientationIMU(){
        limelight.updateRobotOrientation(getYaw());
    }
    public void updateAtHeight(double heightOfLauncher){
        atHeight -= heightOfLauncher;
    }


    /** Motif */
    public void updateMotifCode(){ //Could possibly see oblisk and motif at the same time

        List<LLResultTypes.FiducialResult> fiducials = getResults().getFiducialResults();

        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == 21){
                motifCode = new String[]{"Green","Purple","Purple"};
            }

            else if (f.getFiducialId() == 22){
                motifCode = new String[]{"Purple","Green","Purple"};
            }

            else if (f.getFiducialId() == 23){
                motifCode = new String[]{"Purple","Purple","Green"};
            }
        }
    }
    public String[] getMotifCode(){
        return motifCode;
    }
    public boolean foundMotif(){
        return motifCode != null;
    }

    /** Alliance */
    public String getAlliance(){
        return alliance;
    }

}

