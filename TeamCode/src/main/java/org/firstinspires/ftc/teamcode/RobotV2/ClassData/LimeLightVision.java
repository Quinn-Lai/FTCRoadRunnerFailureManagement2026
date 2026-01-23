package org.firstinspires.ftc.teamcode.RobotV2.ClassData;

import android.util.Size;

import com.acmerobotics.roadrunner.Pose2d;
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
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import javax.crypto.ExemptionMechanism;

public class LimeLightVision {

    /** Hardware */
    private Limelight3A limelight;
    private PinpointLocalizer localizer; //TODO this localizer needs to be updated more often

    /** Variables */
    private double atHeight; //Meters
    public static String[] motifCode;
    public static boolean isFoundMotif = false;
    private String alliance;
    private Telemetry telemetry;
    private boolean activatedLocalizer;

    /** Limelight Variables */
    private double lensHeight;
    private double levelLensAtHeight;


    //----------------------------------------
    /** Constructor */
    public LimeLightVision(HardwareMap hardwareMap, Telemetry telemetry, String alliance){

        /** Hardware Init */
        activatedLocalizer = false;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        //        if (!autoActive){
//            /** Pinpoint Init */
//            imu = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//
//            GoBildaPinpointDriver.Register[] defaultRegisters = {
//                    GoBildaPinpointDriver.Register.DEVICE_STATUS,
//                    GoBildaPinpointDriver.Register.LOOP_TIME,
//                    GoBildaPinpointDriver.Register.X_ENCODER_VALUE,
//                    GoBildaPinpointDriver.Register.Y_ENCODER_VALUE,
//                    GoBildaPinpointDriver.Register.X_POSITION,
//                    GoBildaPinpointDriver.Register.Y_POSITION,
//                    GoBildaPinpointDriver.Register.H_ORIENTATION,
//                    GoBildaPinpointDriver.Register.X_VELOCITY,
//                    GoBildaPinpointDriver.Register.Y_VELOCITY,
//                    GoBildaPinpointDriver.Register.H_VELOCITY,
//            };
//
//            GoBildaPinpointDriver.Register[] onlyPosition = {
//                    GoBildaPinpointDriver.Register.DEVICE_STATUS,
//                    GoBildaPinpointDriver.Register.X_POSITION,
//                    GoBildaPinpointDriver.Register.Y_POSITION,
//                    GoBildaPinpointDriver.Register.H_ORIENTATION,
//            };
//
//            imu.setBulkReadScope(defaultRegisters);
//            imu.setErrorDetectionType(GoBildaPinpointDriver.ErrorDetectionType.CRC);
//
//            imu.resetPosAndIMU();
//        }

//        imu.setOffsets(-84.0, -168.0, DistanceUnit.MM); //TODO Tune offsets, Might not need tho
//        imu.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        imu.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /** Alliance */
        this.alliance = alliance;

        if (this.alliance.equals("blue")) limelight.pipelineSwitch(0);
        else limelight.pipelineSwitch(1);

        /** Variable Init */
        this.atHeight = 0.756015; //approximation 0.756015
//      motifCode = new String[]{"Empty","Empty","Empty"};
        this.telemetry = telemetry;

        lensHeight = 0.24;
        levelLensAtHeight = atHeight - lensHeight;
    }

    //----------------------------------------

    /** Limelight Object */
    public void setLimelightLocalizier(PinpointLocalizer localizer){
        activatedLocalizer = true;
        this.localizer = localizer;
    }
    public void initLimeLight(){
        limelight.start();
    }
    public void killLimeLight(){
        limelight.close();
    }

    /** April Tags */
    public LLResult getResults(){
        return limelight.getLatestResult();
    }
    public double getTx(){
        return getResults().getTx();
    }
    public double getTy(){ //TODO Might cause error if see two april tags (could blacklist oblisk after it sees it or just check id)
        return getResults().getTy();
    }
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
    public Pose2d getCurrentPosLimelight(){
        if (getResults().isValid() && getResults() != null){
            return new Pose2d(0,0,0); //TODO emergency
        }

        //TODO upload field to file
        return new Pose2d(getResults().getBotpose().getPosition().x,getResults().getBotpose().getPosition().y, localizer.getHeadingLocalizerDegrees());

    }
    public double getYaw(){ //TODO might not be the same as april tag library pipline
        return localizer.getHeadingLocalizerDegrees();
    }
    public double getFidYaw(){
        if (canSeeSomeAT()){
            List<LLResultTypes.FiducialResult> fiducials = getResults().getFiducialResults();

            for (LLResultTypes.FiducialResult f : fiducials) {
                if (f.getFiducialId() != 21 && f.getFiducialId() != 22 && f.getFiducialId() != 23){
                    return f.getTargetXDegreesNoCrosshair(); //+ f.getTargetPoseRobotSpace().getOrientation().getYaw(AngleUnit.DEGREES)
                    //f.getRobotPoseFieldSpace().getOrientation().getYaw()
                }
            }
        }

        return 0;
    }
    public double getDisp(){
        if (getResults().isValid() && getResults() != null){
            return (levelLensAtHeight / Math.tan(Math.toRadians(getTyFidDeg()+10))) + RobotConstantsV2.LIMELIGHT_TURRET_DIFFERENCE; //Another triangle here upsidedown
        }
        return 0;
    }
    public boolean canSeeSomeAT(){
        return getResults().isValid() && getResults() != null;
    }

    /** Last Updates */
    public void telemetryLimeLight(){
        if (canSeeSomeAT()) {
            telemetry.addData("tX", getTx());
            telemetry.addData("tY", getTy());
            //telemetry.addData("Disp: ", getDisp());
            telemetry.addData("Motif: ", motifCode);
        }
    }
    public void updateOrientationIMU(){
        if(activatedLocalizer) limelight.updateRobotOrientation(getYaw());
    }
    public void updateAtHeight(double heightOfLauncher){
        atHeight -= heightOfLauncher;
    }

    /** Motif */
    public void updateMotifCode(){ //Could possibly see oblisk and motif at the same time

        List<LLResultTypes.FiducialResult> fiducials = getResults().getFiducialResults();

        double greatestArea = 0;
        LLResultTypes.FiducialResult greatestAreaMotif = null;

        for (LLResultTypes.FiducialResult f : fiducials) {
            double area = f.getTargetArea();
            if (area > greatestArea && (f.getFiducialId() == 21 || f.getFiducialId() == 22 || f.getFiducialId() == 23)){
                greatestArea = area;
                greatestAreaMotif = f;
            }
        }

        if (greatestAreaMotif == null) return;

        if (greatestAreaMotif.getFiducialId() == 21){
            isFoundMotif = true;
            motifCode = new String[]{"Green","Purple","Purple"};
        }

        else if (greatestAreaMotif.getFiducialId() == 22){
            isFoundMotif = true;
            motifCode = new String[]{"Purple","Green","Purple"};
        }

        else if (greatestAreaMotif.getFiducialId() == 23){
            isFoundMotif = true;
            motifCode = new String[]{"Purple","Purple","Green"};
        }

    }
    public static void failsafeMotif(){
        motifCode = new String[]{"Green","Purple","Purple"};
    }
    public String[] getMotifCode(){
        return motifCode;
    }
    public boolean foundMotif(){
        return isFoundMotif;
    }

    /** Alliance */
    public String getAlliance(){
        return alliance;
    }
    public void updateAlliance(String alliance){
        this.alliance = alliance;
        if (this.alliance.equals("blue")) limelight.pipelineSwitch(0);
        else limelight.pipelineSwitch(1);
    }

    public void switchAlliance(){
        if (alliance.equals("blue")){
            alliance = "red";
            limelight.pipelineSwitch(1);

        }

        else{
            alliance = "blue";
            limelight.pipelineSwitch(0);
        }
    }

    public void forceAllianceRed(){
        alliance = "red";
        limelight.pipelineSwitch(1);
    }

    public void forceAllianceBlue(){
        alliance = "blue";
        limelight.pipelineSwitch(0);
    }

}

