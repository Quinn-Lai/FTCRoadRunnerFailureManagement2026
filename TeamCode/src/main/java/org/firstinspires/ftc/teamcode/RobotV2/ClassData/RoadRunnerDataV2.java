package org.firstinspires.ftc.teamcode.RobotV2.ClassData;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

//Holds the data for RoadRunner in Autonomous and TeleOp, same robot object should be used
//Can create methods for TeleOp in RobotData and then call them in RoadRunner Data classes
public class RoadRunnerDataV2{

    private Pose2d beginPose;
    private MecanumDrive drive;
    private RobotDataV2 robot;
    private ArrayList<Action> trajectoryBuilt;
    private FtcDashboard dash;
    private TelemetryPacket packet;
    private List<Action> teleOpActions;
    public static Pose2d lastAutoPosition = null; //TODO maybe issue
    private boolean notAutoPositionStored;
    private Pose2d parkingSpotBlue;
    private Pose2d parkingSpotRed;

//    public RoadRunnerData(HardwareMap hardwareMap) {
//        robot = new RobotData(hardwareMap);
//    }

    //Note to Future Me: Can either create a new robot object for roadrunner actions or use a pre-existing one to stay consistent

    //----------------------------------------
    public RoadRunnerDataV2(RobotDataV2 robot){
        this.robot = robot;

        teleOpActions = new ArrayList<>();
        trajectoryBuilt = new ArrayList<Action>();
        beginPose = null;
        notAutoPositionStored = false;

        parkingSpotBlue = new Pose2d(36.7,32.5,0);
        parkingSpotRed= new Pose2d(36.7,-32.5,0);
    }

    //----------------------------------------

    //Trajectories

    public Pose2d getBeginPose() {
        return beginPose;
    }
    public void setBeginPoseFromAuto(){
        if (lastAutoPosition != null){
            notAutoPositionStored = false;
            beginPose = lastAutoPosition;
        }
        else{
            beginPose = new Pose2d(0,0,0);
            notAutoPositionStored = true;
        }
    }
    public boolean getNotAutoPosStored(){
        return notAutoPositionStored;
    }
    public void setBeginPose(Pose2d beginPose) {
        this.beginPose = beginPose;
    }

    //----------------------------------------

    //Classes

    public RobotDataV2 getRobot() {
        return robot;
    }
    public void updateRobot(RobotDataV2 robot) {
        this.robot = robot;
    }
    public MecanumDrive getDrive() {
        return drive;
    }
    public void createDrive() {
        if (beginPose != null){
            drive = new MecanumDrive(robot.getHardwareMap(), beginPose);
        }
        //Default to 0 if beginPose isn't specified
        else{
            drive = new MecanumDrive(robot.getHardwareMap(), new Pose2d(0,0,0));
        }
    }

    //----------------------------------------

    //Dashboard

    public void createDashboard(){
        dash = FtcDashboard.getInstance();
    }

    public FtcDashboard getDashboard(){
        return dash;
    }
    public void updateTelemetryPacket(){
        packet = new TelemetryPacket();
    }

    //----------------------------------------

    //Create paths

    public Action getParkingTrajectory(Pose2d currentPos, String alliance){

        Pose2d parkingSpot;

        if (alliance.equals("blue")) parkingSpot = parkingSpotBlue;
        else parkingSpot = parkingSpotRed;

        TrajectoryActionBuilder park = getDrive().actionBuilder(currentPos)
                //TODO could set a tangent if calcualted where to go
                .splineToLinearHeading(parkingSpot, 0);

        Action finalpark = park.build();

        return finalpark;
    }

    public Action getAlignTrajectory(Pose2d currentPos, double limelightYaw){

        limelightYaw *= -1;

        TrajectoryActionBuilder park = getDrive().actionBuilder(currentPos)
                .turn(limelightYaw);

        Action finalTurn = park.build();

        return finalTurn;

    }

//    public SequentialAction getParkingSeqAction(Pose2d currentPos, String alliance){
//        return new SequentialAction(
//                getParkingTrajectory(currentPos, alliance)
//        );
//    }
    public void addTeleOpAction(Action action){
        teleOpActions.add(action);
    }

    public void killTeleOpActions(){
        teleOpActions.clear();
    }
    public void runTeleOpActions(){

        updateTelemetryPacket();

        List<Action> newActions = new ArrayList<>();
        for (Action a:teleOpActions){
            a.preview(packet.fieldOverlay());
            if (a.run(packet)){
                newActions.add(a);
            }
        }

        teleOpActions = newActions; //Presumably to Empty Old Array

        dash.sendTelemetryPacket(packet);

    }

    //----------------------------------------

    //Build Trajectory

    //Build Trajectory Path Array List
    public ArrayList<Action> getTrajectoryPath() {
        return trajectoryBuilt;
    }
    public void addTrajectory(Action traj) {
        trajectoryBuilt.add(traj);
    }

    //Accesses Build Trajectory from ArrayList
    public Action getTrajectory(int pathNum){
        if (pathNum > 0 && pathNum <= trajectoryBuilt.size()){
            return trajectoryBuilt.get(pathNum-1);
        }
        System.out.println("Out of Bounds Trajectory");
        return null; //Will Throw an Error on Purpose
    }

    //ChangesTrajectory from ArrayList (Uses TrajectoryActionBuilder and converts to Action with build)
    public Action changeTrajectory(int pathNum, TrajectoryActionBuilder traj){
        if (pathNum > 0 && pathNum <= trajectoryBuilt.size()){
            return trajectoryBuilt.set(pathNum-1,traj.build());
        }
        System.out.println("Out of Bounds Trajectory");
        return null; //Will Throw an Error on Purpose
    }

    //Creates ArrayList of trajectories
    public void createTrajectoryPath(TrajectoryActionBuilder[] trajectoryUpload){

        if (!trajectoryBuilt.isEmpty()){
            trajectoryBuilt.clear();
        }

        for (TrajectoryActionBuilder t : trajectoryUpload){
            trajectoryBuilt.add(t.build());
        }
    }


    //----------------------------------------

    //Robot Trajectories Methods

    public class Reminder implements Action{ //Creates Class, so each class is represented as an Action

        private double x;
        public Reminder(double x){
            this.x = x;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //Commands & Methods Go Here

            robot.getDriveTrain().getLFmotor().setPower(x);

            return false;
        }
    }
    public Action doReminder(double x) { //Creates method to return the action as an object
            return new Reminder(x); //Constructor
    }
    //Action test = new Reminder(1); //Does the same thing as above
    //(Modified) Sequential Robot Trajectories Using Methods
    public SequentialAction getSequentialConcept(){

        SequentialAction concept = new SequentialAction(
                //Other Method
                new SleepAction(10)
                //Other Method
        );

        return concept;

    }

}
