package org.firstinspires.ftc.teamcode.ClassData;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

@Config
public class OpenCVData {
    private RotatedRect boxFit;
    private double centerX;
    private double centerY;
    private HardwareMap hardwareMap;

    private VisionPortal portal;
    private ColorBlobLocatorProcessor colorLocator;

    public OpenCVData(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
    }

    public void createColorLocator(ColorRange color){
         colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(color)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                //.setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setRoi(ImageRegion.asImageCoordinates(50,50,270,150))
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();
    }

    public void createVisionPortal(){
         portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
    }




    public ColorBlobLocatorProcessor getColorLocator(){
        return colorLocator;
    }

    public double getX(){
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);  // filter out very small blobs.

        for(ColorBlobLocatorProcessor.Blob b : blobs)
        {
            boxFit = b.getBoxFit();
            centerX = boxFit.center.x;
        }

        return centerX;
    }

    public double getY(){
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);  // filter out very small blobs.

        for(ColorBlobLocatorProcessor.Blob b : blobs)
        {
            boxFit = b.getBoxFit();
            centerY = boxFit.center.y;
        }

        return centerY;
    }

    public double getXError(){
        return getX()-160;
    }

    public double getPosError(){
        return getX() - 160;
        //telemetry.addData("Error:", error);
        //telemetry.addData("Real World:", error / errorConversion);
        //Error only acounting for specific decimation (around 6 inches)
        //Can possibly fix with quadratic equation calibration but idk
    }

    public double getWidth(){
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);  // filter out very small blobs.

        org.opencv.core.Size myBoxFitSize;
        double width = 0;

        for(ColorBlobLocatorProcessor.Blob b : blobs)
        {
            boxFit = b.getBoxFit();
            myBoxFitSize = boxFit.size;
            width = myBoxFitSize.width;
        }
        return width;
    }

    public double getHeight(){
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);  // filter out very small blobs.

        org.opencv.core.Size myBoxFitSize;
        double height = 0;

        for(ColorBlobLocatorProcessor.Blob b : blobs)
        {
            boxFit = b.getBoxFit();
            myBoxFitSize = boxFit.size;
            height = myBoxFitSize.height;
        }
        return height;
    }

    public double getAngle(){
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);  // filter out very small blobs.

        org.opencv.core.Size myBoxFitSize;
        double angle = 0;

        for(ColorBlobLocatorProcessor.Blob b : blobs)
        {
            boxFit = b.getBoxFit();
            myBoxFitSize = boxFit.size;
            angle = boxFit.angle;
        }
        return angle;
    }

    public void closePortal(){
        portal.close();
    }

    public void telemetryBoxSize(Telemetry telemetry){
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);  // filter out very small blobs.

        org.opencv.core.Size myBoxFitSize;

        for(ColorBlobLocatorProcessor.Blob b : blobs)
        {
            boxFit = b.getBoxFit();
            myBoxFitSize = boxFit.size;
            telemetry.addData("Width:", myBoxFitSize.width);
            telemetry.addData("Height:", myBoxFitSize.height);
            telemetry.addData("Angle:", boxFit.angle);
        }
    }

}