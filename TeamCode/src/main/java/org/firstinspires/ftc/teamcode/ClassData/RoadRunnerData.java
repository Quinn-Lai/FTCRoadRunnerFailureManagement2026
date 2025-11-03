package org.firstinspires.ftc.teamcode.ClassData;

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
public class RoadRunnerData{

    private Pose2d beginPose;
    private MecanumDrive drive;
    private RobotData robot;
    private ArrayList<Action> trajectoryBuilt;
    private FtcDashboard dash;
    private TelemetryPacket packet;
    private List<Action> teleOpActions;

//    public RoadRunnerData(HardwareMap hardwareMap) {
//        robot = new RobotData(hardwareMap);
//    }

    //Note to Future Me: Can either create a new robot object for roadrunner actions or use a pre-existing one to stay consistent

    public RoadRunnerData(RobotData robot){
        this.robot = robot;

        teleOpActions = new ArrayList<>();
        trajectoryBuilt = new ArrayList<Action>();
    }

    //Getters and Setters

    public Pose2d getBeginPose() {
        return beginPose;
    }

    public void setBeginPose(Pose2d beginPose) {
        this.beginPose = beginPose;
    }

    public RobotData getRobot() {
        return robot;
    }

    public void updateRobot(RobotData robot) {
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

    public void createDashboard(){
        dash = FtcDashboard.getInstance();
    }

    public FtcDashboard getDashboard(){
        return dash;
    }
    public void updateTelemetryPacket(){
        packet = new TelemetryPacket();
    }


    //TeleOp

    public SequentialAction getTestSeqAction(){
        return new SequentialAction(
                new InstantAction(() -> robot.getDriveTrain().getLFmotor().setPower(0.3)),
                new SleepAction(1),
                new InstantAction(() -> robot.getDriveTrain().getLFmotor().setPower(0))
        );
    }
    public void addTeleOpAction(SequentialAction action){
        teleOpActions.add(action);
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


    //------------------------------------------------------

    //Robot Trajectories Methods

    //------------------------------------------------------

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

    //------------------------------------------------------

    //(Modified) Sequential Robot Trajectories Using Methods

    //------------------------------------------------------

    public SequentialAction getSequentialConcept(){

        SequentialAction concept = new SequentialAction(
                //Other Method
                new SleepAction(10)
                //Other Method
        );

        return concept;

    }

}
