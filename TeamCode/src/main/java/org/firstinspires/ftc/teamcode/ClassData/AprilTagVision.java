package org.firstinspires.ftc.teamcode.ClassData;

/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagVision {

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;
    private HardwareMap hardwareMap;
    private WebcamName webcam;
    private Telemetry telemetry;
    private double atHeight; //Meters
    private String[] motifCode;
    private final double horizontalDist = 0.12065;
    private final double yCamTri = 0.0381;
    private int idDesired;
    private String alliance;

    public AprilTagVision(HardwareMap hardwareMap, Telemetry telemetry, String alliance){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.atHeight = 0.756015; //approximation 0.756015

        webcam = hardwareMap.get(WebcamName.class,"Webcam 1");

        if (alliance. equals("blue")){
            idDesired = 20;
        }
        else{
            idDesired = 24;
        }
        this.alliance = alliance;
    }


    public void updateAtHeight(double heightOfLauncher){
        atHeight -= heightOfLauncher;
    }
    public void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(1);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(webcam);

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }

    public void activeVisionPortal(boolean active){
        visionPortal.setProcessorEnabled(aprilTag, active);
    }

    public void closeVisionPortal(){
        visionPortal.close();
    }

    public void telemetryAprilTag() {

        if (canSeeAT()){
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {

                //Here sort through april tags to find out which motif is which

                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (meters)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (degree, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }

            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");
        }
    }

    public boolean canSeeAT(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == idDesired){
                return true;
            }
        }

        //return !(currentDetections.isEmpty());
        return false;
    }

    public double getDisp(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        double disp = 0;
        double c = 0;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == idDesired){ //Red Blue
                double range = detection.ftcPose.range * 100;

                //range^2 = height of apriltag^2 + xHorizontal^2

                disp = Math.sqrt(Math.pow(range,2) - Math.pow(atHeight,2)) + yCamTri;

                c = Math.sqrt(Math.pow(disp,2) - Math.pow(horizontalDist,2));

                break;
            }
        }
        return c;
    }

    public void updateMotifCode(){ //Could possibly see oblisk and motif at the same time
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {

            if (detection.id == 21){
                motifCode = new String[]{"G","P","P"};
            }

            else if (detection.id == 22){
                motifCode = new String[]{"P","G","P"};
            }

            else if (detection.id == 23){
                motifCode = new String[]{"P","P","G"};
            }
        }
    }

    public String[] getMotifCode(){

        if (motifCode == null){
            return new String[]{};
        }

        return motifCode;
    }

    public double getYaw(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        double yaw = 0;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == idDesired){ //Red Blue
                yaw = detection.ftcPose.yaw;
                break;
            }
        }

        return yaw;

    }

    public double getXPos(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        double x = 0;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == idDesired){ //Red Blue
                x = detection.ftcPose.x;
                break;
            }
        }

        return x;

    }

    public String getAlliance(){
        return alliance;
    }

}

