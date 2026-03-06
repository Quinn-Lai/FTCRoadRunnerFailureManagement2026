package org.firstinspires.ftc.teamcode.RobotV2.ClassData;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

//Holds the data for RoadRunner in Autonomous and TeleOp, same robot object should be used
//Can create methods for TeleOp in RobotData and then call them in RoadRunner Data classes
public class RoadRunnerDataV2{

    //Localization
    private Pose2d beginPose;
    private MecanumDrive drive;

    //Robot Parts
    public RobotDataV2 robotData;

    //Trajectories
    private ArrayList<Action> trajectoryBuilt;
    private FtcDashboard dash;
    private ElapsedTime failsafeTimer; //For intaking part
    private boolean loopingActive; //For turret
    private boolean isDoneInit;
    private double displacement;
    private double intendedHeading;
    private boolean openGateActive;

    //TeleOp
    private List<Action> teleOpActions;
    public static Pose2d lastAutoPosition = null;
    public static boolean isAutoPosStored = false;
    private Action currentTeleOpAction;
    private boolean teleOpActionActive;
    private boolean isCaseOhActive;
    private boolean flagIntake;

    //----------------------------------------
    public RoadRunnerDataV2(RobotDataV2 robot){

        drive = null;
        RobotConstantsV2.AUTO_FAILSAFE_TIMER = RobotConstantsV2.AUTO_FAILSAFE_TIMER_CLOSE;

        this.robotData = robot;
        loopingActive = false;
        isDoneInit = false;
        teleOpActionActive = false;
        openGateActive = false;

        teleOpActions = new ArrayList<>();
        trajectoryBuilt = new ArrayList<Action>();
        beginPose = new Pose2d(0,0,0);

        failsafeTimer = new ElapsedTime();

        flagIntake = false;
        currentTeleOpAction = null;
        displacement = 0;
        intendedHeading = 0;
        isCaseOhActive = false;

        setBeginPoseFromAuto();
    }

    //----------------------------------------

    public boolean getFlagIntake(){
        return flagIntake;
    }

    /** Localization */
    public Pose2d getBeginPose() {
        return beginPose;
    }
    public void setBeginPose(Pose2d beginPose) {
        this.beginPose = beginPose;
    }
    public MecanumDrive getDrive() {
        return drive;
    }
    public void createDrive() {
        if (beginPose != null){
            drive = new MecanumDrive(robotData.getHardwareMap(), beginPose);
        }
        //Default to 0 if beginPose isn't specified
        else{
            drive = new MecanumDrive(robotData.getHardwareMap(), new Pose2d(0,0,0));
        }
    }
    public void updateDrivePos(Pose2d update){
        drive.localizer.setPose(update);
    }
    public void createDashboard(){
        dash = FtcDashboard.getInstance();
    }
    public FtcDashboard getDashboard(){
        return dash;
    }

    //Sub this in for vx0 in physics calc
    public double getNetVelocity(){
        return Math.sqrt(Math.pow(getVelocityY(),2) + Math.pow(getVelocityX(),2));
    }
    public double getVelocityX(){
        return drive.localizer.update().linearVel.x * 0.0254;
    }

    public double getVelocityY(){

        //double error = getYawLocalizationOld(alliance) - getYaw360();
        return drive.localizer.update().linearVel.y * 0.0254; //* Math.cos(error);
    }

    //Global Yaw
    public double getYaw(){
        //return drive.getLocalizerPinpoint().getHeadingLocalizerDegrees();
        return Math.toDegrees(drive.localizer.getPose().heading.toDouble());
    }
    public double getYawRad(){
        return drive.localizer.getPose().heading.toDouble();
    }
    public double getYaw360(){
        return (getYaw() + 360) % 360;
    }

    //Localization (From Goal)
    public double getDispLocalization(String alliance) {

        if (alliance.equals("blue")) {
            return 0.0254 * (Math.sqrt(Math.pow(RobotConstantsV2.GLOBAL_GOAL_AIM_BLUE[0] - drive.localizer.getPose().position.x, 2) +  Math.pow(RobotConstantsV2.GLOBAL_GOAL_AIM_BLUE[1] - drive.localizer.getPose().position.y, 2)));
        }
        else {
            return 0.0254 * (Math.sqrt(Math.pow(RobotConstantsV2.GLOBAL_GOAL_AIM_RED[0] - drive.localizer.getPose().position.x, 2) +  Math.pow(RobotConstantsV2.GLOBAL_GOAL_AIM_RED[1] - drive.localizer.getPose().position.y, 2)));
        }
    }

    public double getRobotTheta(){
        double degrees = Math.toDegrees(Math.atan2(getVelocityY(),getVelocityX()));

        if (degrees < 0) degrees += 360;

        return degrees;
    }

    //TODO: HAHAHHA Im SO SMart, polar coordinates go burr (calc bc for life hahaiuehfuikahse)
    public double getNormXVel(){
        return getNetVelocity() * Math.cos(Math.toRadians(getYaw360()));
    }

    public double getNormYVel(){
        return  getNetVelocity() * Math.sin(Math.toRadians(getYaw360()));
    }

    public Double[] getVelTowerSurface(String alliance){

        double theta;
        if (alliance.equals("blue")){
            theta = 50;
        }
        else{
            theta = 140;
        }

        double vel = 0;
        double thetaDiff = 0;

        if (getRobotTheta() >= theta){
            thetaDiff = getRobotTheta() - theta;
            vel = getNormXVel() * Math.cos(Math.toRadians(thetaDiff));

        }

        else{
            vel = getNormYVel() * Math.sin(Math.toRadians(getRobotTheta()));
        }

        return new Double[]{vel * Math.cos(Math.toRadians(theta)), vel * Math.sin(Math.toRadians(theta))};

    }

    public double getMovementTowerCoordsX(String alliance, double timeExtrema){
        return RobotConstantsV2.GLOBAL_GOAL_POS_BLUE[0] + getVelTowerSurface(alliance)[0] * timeExtrema;
    }

    public double getMovementTowerCoordsY(String alliance, double timeExtrema){
        return RobotConstantsV2.GLOBAL_GOAL_POS_BLUE[1] + getVelTowerSurface(alliance)[1] * timeExtrema; //TODO Times by time extrema value
    }

    public double getYawLocalization(String alliance, double time){
        double heading;

        if (alliance.equals("blue")) {

//            robotData.getTelemetry().addData("X: ",  drive.localizer.getPose().position.x);
//            robotData.getTelemetry().addData("Y: ",  drive.localizer.getPose().position.y);
//
//            robotData.getTelemetry().addData("X Goal: ", RobotConstantsV2.GLOBAL_GOAL_POS_BLUE[0] );
//            robotData.getTelemetry().addData("Y Goal: ", RobotConstantsV2.GLOBAL_GOAL_POS_BLUE[1] );
//
//            robotData.getTelemetry().addData("Pos: ", Math.toDegrees(Math.atan2(RobotConstantsV2.GLOBAL_GOAL_POS_BLUE[1] - drive.localizer.getPose().position.y, RobotConstantsV2.GLOBAL_GOAL_POS_BLUE[0] - drive.localizer.getPose().position.x)));
            heading = Math.toDegrees(Math.atan2(RobotConstantsV2.GLOBAL_GOAL_POS_BLUE[1] - drive.localizer.getPose().position.y, RobotConstantsV2.GLOBAL_GOAL_POS_BLUE[0] - drive.localizer.getPose().position.x));
        }
        else {
            heading = Math.toDegrees(Math.atan2(RobotConstantsV2.GLOBAL_GOAL_POS_RED[1] - drive.localizer.getPose().position.y, RobotConstantsV2.GLOBAL_GOAL_POS_RED[0] - drive.localizer.getPose().position.x));
        }

        if (heading < 0) heading += 360;

        return heading;
    }

    public double getYawLocalizationOld(String alliance, double timeExtrema){
        double heading;

        if (alliance.equals("blue")) {

//            robotData.getTelemetry().addData("X: ",  drive.localizer.getPose().position.x);
//            robotData.getTelemetry().addData("Y: ",  drive.localizer.getPose().position.y);
//
//            robotData.getTelemetry().addData("X Goal: ", RobotConstantsV2.GLOBAL_GOAL_POS_BLUE[0] );
//            robotData.getTelemetry().addData("Y Goal: ", RobotConstantsV2.GLOBAL_GOAL_POS_BLUE[1] );
//
//            robotData.getTelemetry().addData("Pos: ", Math.toDegrees(Math.atan2(RobotConstantsV2.GLOBAL_GOAL_POS_BLUE[1] - drive.localizer.getPose().position.y, RobotConstantsV2.GLOBAL_GOAL_POS_BLUE[0] - drive.localizer.getPose().position.x)));
            heading = Math.toDegrees(Math.atan2(getMovementTowerCoordsY(alliance, timeExtrema) - drive.localizer.getPose().position.y, getMovementTowerCoordsX(alliance, timeExtrema) - drive.localizer.getPose().position.x));
        }
        else {
            heading = Math.toDegrees(Math.atan2(getMovementTowerCoordsY(alliance, timeExtrema) - drive.localizer.getPose().position.y, getMovementTowerCoordsX(alliance, timeExtrema) - drive.localizer.getPose().position.x));
        }

        if (heading < 0) heading += 360;

        return heading;
    }
    public void updateGlobalRobotPosition(LimeLightVision limelight, boolean localOn){

        //drive.updatePoseEstimate();
        //double yaw = getYaw();

//        if (limelight.canSeeSomeAT() && localOn){
////            limelight.updateGlobalPosMetaTag();
////            Pose2d pose = RobotConstantsV2.LAST_ROBOT_POS;
////            TelemetryPacket packet = new TelemetryPacket();
////            packet.fieldOverlay().setStroke("#3F51B5");
////            Drawing.drawRobot(packet.fieldOverlay(), pose);
////            FtcDashboard.getInstance().sendTelemetryPacket(packet);
////            updateDrivePos(pose);
//            yaw = limelight.getYawMT1();
//        }

        if (limelight.canSeeSomeAT() && localOn){

            //limelight.updateOrientationIMU(yaw1);
            limelight.updateGlobalPosMetaTag();
            Pose2d pose = RobotConstantsV2.LAST_ROBOT_POS;
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            updateDrivePos(pose);
            //drive.updatePoseEstimate();
        }
        else{
            Pose2d pose = drive.localizer.getPose();
            RobotConstantsV2.LAST_ROBOT_POS = pose;
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            drive.updatePoseEstimate();
        }
    }

    //Totality
    public double getYawTotality(LimeLightVision limelight){
        if (limelight.canSeeSomeAT()) return limelight.getFidYaw();
        else return getYawLocalization(limelight.getAlliance(),0);
    }
    public double getDispTotality(LimeLightVision limelight){
        if (limelight.canSeeSomeAT()) return limelight.getDisp();
        else return getDispLocalization(limelight.getAlliance());
    }

    /** TeleOp */
    public void setBeginPoseFromAuto(){
        if (isAutoPosStored){
            beginPose = lastAutoPosition;
        }
        else{
            beginPose = new Pose2d(0,0,0); //Failsafe
        }

        isAutoPosStored = false;
    }
    public void setCurrentTeleOpAction(Action a){
        currentTeleOpAction = a;
    }
    public void endTeleOpAction(){
        currentTeleOpAction = null;
    }
    public void setTeleOpActionActive(boolean active){
        this.teleOpActionActive = active;
    }
    public boolean isTeleOpActionActive(){
        return teleOpActionActive;
    }
    public void addTeleopAction(Action a){
        teleOpActions.add(a);
    }
    public void runCurrentTeleOpAction(){

        TelemetryPacket packet = new TelemetryPacket();

        List<Action> newActions = new ArrayList<>();

        for (Action action : teleOpActions){
            action.preview(packet.fieldOverlay());
            if (action.run(packet)){
                newActions.add(action);
            }
        }

        teleOpActions = newActions;

        dash.sendTelemetryPacket(packet);

    }
    public List<Action> getTeleOpActions(){
        return teleOpActions;
    }
    public void updateTeleOpActions(){

        TelemetryPacket packet = new TelemetryPacket();

        List<Action> newActions = new ArrayList<>();
        for (Action action : teleOpActions) {
            action.preview(packet.fieldOverlay());
            action.run(packet);
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        teleOpActions = newActions;

        dash.sendTelemetryPacket(packet);

    }
    public void addTeleOpAction(Action action){
        teleOpActions.add(action);
    }
    public void killTeleOpActions(){
        teleOpActions.clear();
    }
    public Action generateTrajectory(Pose2d origin, Pose2d destination, double tan1, double tan2){
        return getDrive().actionBuilder(origin)
                .setTangent(Math.toRadians(tan1))
                .splineToLinearHeading(destination,Math.toRadians(tan2))
                .build();
    }


    /** Trajectories */
    public int getTangentThreshold(double threshold){

        //Top
        if (drive.localizer.getPose().position.x < threshold){
            return (int) Math.toRadians(0);
        }

        //Bottom
        else{
            return (int) Math.toRadians(180);
        }

    }
    public Action getParkingTrajectory(String alliance){

        Action park;

        if (alliance.equals("blue")){

            park = getDrive().actionBuilder(drive.localizer.getPose())
                    //TODO Add Tangent Direction of path
                    .setTangent(Math.toRadians(getTangentThreshold(RobotConstantsV2.parkingBlue.position.x))) //0
                    .splineToLinearHeading(RobotConstantsV2.parkingBlue,Math.toRadians(90), new TranslationalVelConstraint(RobotConstantsV2.PARKING_SPEED))
                    .build();
        }
        else{
            park = getDrive().actionBuilder(drive.localizer.getPose())
                    .setTangent(Math.toRadians(getTangentThreshold(RobotConstantsV2.parkingRed.position.x)))
                    .splineToLinearHeading(RobotConstantsV2.parkingRed,Math.toRadians(270), new TranslationalVelConstraint(RobotConstantsV2.PARKING_SPEED))
                    .build();
        }

        return park;
    }
    public Action getAlignTrajectory(LimeLightVision limelight){

        Action align;
        double limelightYaw;

        if (limelight.canSeeSomeAT()){

            limelightYaw = limelight.getYawMetaAdjusted();

            if (limelightYaw == 0) return new InstantAction( () -> doNothing());

            align = getDrive().actionBuilder(drive.localizer.getPose())
                    .turn(Math.toRadians(-limelightYaw))
                    .build();
        }
        else{

            limelightYaw = getYawLocalization(limelight.getAlliance(),0);
            double currentYaw = getYaw360();

            double targetYaw = limelightYaw - currentYaw;

            align = getDrive().actionBuilder(drive.localizer.getPose())
                    .turn(Math.toRadians(-targetYaw))
                    .build();
        }

        return align;

    }
    public Action getAlignTrajectoryAuto(double limelightYaw){

        if (limelightYaw == 0) return new InstantAction( () -> doNothing());

        Action align = getDrive().actionBuilder(getDrive().localizer.getPose())
                .turn(Math.toRadians(-limelightYaw))
                .build();

        return align;

    }
    public Action getCaseOhStall(){

        Action caseOh = getDrive().correctionActionBuilder(drive.localizer.getPose())
                .splineToLinearHeading(new Pose2d(drive.localizer.getPose().position.x+0.01,drive.localizer.getPose().position.y,drive.localizer.getPose().heading.toDouble()), 0, new TranslationalVelConstraint(10))
                .build();

        return caseOh;

    }
    public Action getGateIntakeTrajectory(String alliance){

        Action gate;

        if (alliance.equals("blue")){

            gate = getDrive().actionBuilder(drive.localizer.getPose())
                    //TODO Add Tangent Direction of path
                    .setTangent(Math.toRadians(getTangentThreshold(RobotConstantsV2.GATE_INTAKE_BLUE.position.x))) //270
                    .splineToLinearHeading(RobotConstantsV2.GATE_INTAKE_BLUE,Math.toRadians(270), new TranslationalVelConstraint(RobotConstantsV2.PARKING_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED))
                    .build();
        }
        else{
            gate = getDrive().actionBuilder(drive.localizer.getPose())
                    .setTangent(Math.toRadians(getTangentThreshold(RobotConstantsV2.GATE_INTAKE_RED.position.x))) //90
                    .splineToLinearHeading(RobotConstantsV2.GATE_INTAKE_RED,Math.toRadians(90), new TranslationalVelConstraint(RobotConstantsV2.PARKING_SPEED), new ProfileAccelConstraint(RobotConstantsV2.MIN_ACCEL_SPEED,RobotConstantsV2.MAX_ACCEL_SPEED))
                    .build();
        }

        return gate;
    }

    public boolean isCaseOhActive(){
        return isCaseOhActive;
    }

    public void switchCaseOh(){
        if (isCaseOhActive) isCaseOhActive = false;
        else{
            requestCaseOhStall();
            isCaseOhActive = true;
        }
    }

    public void forceStopCaseOh(){
        isCaseOhActive = false;
    }

    public void forceOnCaseOh(){
        requestCaseOhStall();
        isCaseOhActive = true;
    }

    /** Trajectories Request*/
    public void requestPark(String alliance){
        if (!teleOpActionActive){
            robotData.getDriveTrain().killAllWheels();
            teleOpActionActive = true;
            killTeleOpActions();
            addTeleopAction(getParkingTrajectory(alliance));
        }
    }
    public void requestAlign(LimeLightVision limelight){

        if (!teleOpActionActive){
            robotData.getDriveTrain().killAllWheels();
            teleOpActionActive = true;
            killTeleOpActions();
            addTeleopAction(getAlignTrajectory(limelight));
        }
    }
    public void requestCaseOhStall(){

        if (!teleOpActionActive){
            robotData.getDriveTrain().killAllWheels();
            teleOpActionActive = true;
            killTeleOpActions();
            addTeleopAction(getCaseOhStall());
        }
    }
    public void requestGateIntake(String alliance){
        if (!teleOpActionActive){
            robotData.getDriveTrain().killAllWheels();
            robotData.getCarosel().forceIntakeOn();
            teleOpActionActive = true;
            killTeleOpActions();
            addTeleopAction(getGateIntakeTrajectory(alliance));
        }
    }


    /** Robot Parts */
    public RobotDataV2 getRobotData() {
        return robotData;
    }
    public void updateRobotData(RobotDataV2 robotData){
        this.robotData = robotData;
    }
    public void setLoopStatus(boolean active){
        this.loopingActive = active;
    }
    public boolean isLoopingActive(){
        return loopingActive;
    }

    /** Trajectory Generation */
    public ArrayList<Action> getTrajectoryPath() {
        return trajectoryBuilt;
    }     //Build Trajectory Path Array List
    public void addTrajectory(Action traj) {
        trajectoryBuilt.add(traj);
    }
    public Action getTrajectory(int pathNum){
        if (pathNum > 0 && pathNum <= trajectoryBuilt.size()){
            return trajectoryBuilt.get(pathNum-1);
        }
        System.out.println("Out of Bounds Trajectory");
        return null; //Will Throw an Error on Purpose
    }
    public Action changeTrajectory(int pathNum, TrajectoryActionBuilder traj){
        //ChangesTrajectory from ArrayList (Uses TrajectoryActionBuilder and converts to Action with build)
        if (pathNum > 0 && pathNum <= trajectoryBuilt.size()){
            return trajectoryBuilt.set(pathNum-1,traj.build());
        }
        System.out.println("Out of Bounds Trajectory");
        return null; //Will Throw an Error on Purpose
    }
    public void createTrajectoryPath(TrajectoryActionBuilder[] trajectoryUpload){

        if (!trajectoryBuilt.isEmpty()){
            trajectoryBuilt.clear();
        }

        for (TrajectoryActionBuilder t : trajectoryUpload){
            trajectoryBuilt.add(t.build());
        }
    }
    public void setDisplacement(double displacement){
        this.displacement = displacement;
    }
    public double getDisplacement(){
        return displacement;
    }


    /** Quick Actions */

    public Action setDisplacementQuick(double disp){
        return new InstantAction( () -> setDisplacement(disp));
    }

    public Action updateLastPosition(){
        RoadRunnerDataV2.isAutoPosStored = true;
        return new InstantAction( () -> RoadRunnerDataV2.lastAutoPosition = getDrive().localizer.getPose()); //Get last pos)
    }
    public Action intakeReverse(){
        return new InstantAction (() -> robotData.getCarosel().forceReverseIntakeOn());
    }
    public Action updateTelemetryIn(Telemetry telemetry){
        return new InstantAction( () -> robotData.updateTelemetry(telemetry));
    }
    public Action intakeOn() {
        return new InstantAction(() -> robotData.getCarosel().forceIntakeOn());
    }
    public Action intakeOff() {
        return new InstantAction(() -> robotData.getCarosel().forceIntakeOff());
    }
    public Action startFailsafeTimer() {
        return new InstantAction(() -> failsafeTimer.reset());
    }
    public Action setLooping(boolean active){
        return new InstantAction(()-> this.loopingActive = active);
    }
    public Action setIsDoneInit(boolean done){
        return new InstantAction(() -> isDoneInit = done);
    }
    public Action updateTelemetry(){
        return new InstantAction(() ->robotData.getTelemetry().update());
    }
    public Action updatePattern(String[] pattern){
        return new InstantAction(() -> robotData.getCarosel().updatePattern(pattern));
    }
    public Action killTurret() {
        return new InstantAction(() -> robotData.getTurret().killShooter());
    }
    public Action switchTransfer() { //Creates method to return the action as an object
        return new InstantAction(() -> robotData.getCarosel().toggleTransfer()); //Constructor
    }
    public Action forceTransferDown() { //Creates method to return the action as an object
        return new InstantAction(() -> robotData.getCarosel().forceTransferDown()); //Constructor
    }
    public Action updatePattern(){
        return new InstantAction(()-> robotData.getCarosel().updatePattern(LimeLightVision.motifCode));
    }
    public Action updateCode(LimeLightVision limeLightVision){
        return new InstantAction(limeLightVision::updateMotifCode);
    }
    public Action checkPointAction(Action action){

        if (action == null) return new InstantAction( () -> doNothing());

        return action;
    }
    public Action cycleFirstPattern(){
        return new InstantAction( () -> robotData.getCarosel().cyclePattern(0));
    }
    public Action cycleFirstSlot(){
        return new InstantAction(() -> robotData.getCarosel().cycleOrigin());
    }
    public Action cycleQuickSlot(boolean patternActive){
        if (patternActive) return cycleFirstPattern();
        else return cycleFirstSlot();
    }
    public Action forceFeedInventory(boolean active,String color1, String color2, String color3){
        if (active) return new InstantAction(() -> robotData.getCarosel().forceFeedInventory(color1,color2,color3));
        return new InstantAction(() -> doNothing());
    }
    public Action forceFeedCycle(boolean active){
        if (active) return cycleFirstSlot();
        return new InstantAction( () -> doNothing());
    }
    public Action quickUpdateMotif(){
        return new InstantAction( () -> robotData.getCarosel().updatePattern(LimeLightVision.motifCode));
    }

    /** Complex Actions */
    public class UpdateCaroselEncdoer implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            robotData.getCarosel().updateCaroselEncoder();

            return loopingActive;

        }

    }
    public Action updateCaroselEncoder(){
        return new UpdateCaroselEncdoer();
    }
    public class CheckAutoIntake implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            robotData.getCarosel().autoIntakeCycle();

            if (failsafeTimer.milliseconds() > RobotConstantsV2.AUTO_FAILSAFE_TIMER){
                flagIntake = true;
                return false;
            }

            flagIntake = false;
            return (robotData.getCarosel().isEmptySpot());
        }
    }
    public Action checkAutoIntake() {
        return new CheckAutoIntake();
    }

    public SequentialAction flagException(SequentialAction action){

        if (flagIntake) return new SequentialAction();

        return action;
    }

    public SequentialAction flagExceptionProc(SequentialAction action){

        if (flagIntake) return action;

        return new SequentialAction();
    }

    public class RequestPatternFire implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            robotData.getDriveTrain().setCurrentSubMode(RobotConstantsV2.subModes[1]);
            robotData.getCarosel().activatePatternInProg();
            robotData.getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
            robotData.getCarosel().setMaxShotsPattern();

            return false;
        }
    }
    public Action requestPatternFire() {
        return new RequestPatternFire();
    }
    public class PatternFireWaitSpeed implements Action {

        public double disp;

        public PatternFireWaitSpeed(double disp){
            this.disp = disp;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robotData.getCarosel().transferReceiveTimer();
            return robotData.getDriveTrain().executePatternFireAutoFar(disp);
//            if (robotData.getCarosel().getSortedFireCurrentShotCount() >= robotData.getCarosel().getMaxShots()) {
//                robotData.getCarosel().transferReceiveTimer();
//                robotData.getCarosel().resetSortedFireCurrentShotCount();
//                robotData.getDriveTrain().endSubMode();
//                return false;
//            }
//
//            switch (robotData.getCarosel().getCurrentSubModeQueue()) {
//                case ("cycle"):
//
//                    if (!robotData.getCarosel().isTransferCooldownActive()) {
//
//                        robotData.getCarosel().cycleSortedFire(robotData.getCarosel().getSortedFireCurrentShotCount());
//
//                        if (robotData.getCarosel().isCaroselInPlace()) {
//                            robotData.getCarosel().resetShotSuccess();
//                            robotData.getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
//                            robotData.getCarosel().activateTransferInProg();
//                        }
//                    }
//
//                    break;
//
//                case ("transfer"):
//
//                    if (robotData.getCarosel().getShotSuccessInstant() || robotData.getCarosel().isFailsafeSubmode()){
//                        robotData.getCarosel().endFailsafeSubmode();
//                        robotData.getCarosel().incrementSortedFireCurrentShotCount();
//                        robotData.getCarosel().activatePatternInProg();
//                        robotData.getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
//                        break;
//                    }
//
//                    boolean status;
//
//                    if (robotData.getTurret().isFarToggled()) status = Math.abs(robotData.getTurret().getTPSError(RobotConstantsV2.FAR_BALL_DISTANCE)) < robotData.getTurret().getTPS(RobotConstantsV2.FAR_BALL_DISTANCE) * RobotConstantsV2.SHOOTER_SPEED_THRESHOLD;
//                    else status = Math.abs(robotData.getTurret().getTPSError(RobotConstantsV2.CLOSE_BALL_DISTANCE)) < robotData.getTurret().getTPS(RobotConstantsV2.CLOSE_BALL_DISTANCE) * RobotConstantsV2.SHOOTER_SPEED_THRESHOLD;
//
//                    //Only Shoot up to speed
//                    if (status) robotData.getCarosel().subModeTransferStartTimer();
//
//                    robotData.getCarosel().checkShotSuccess();
//
//                    break;
//
//                default:
//                    break;
//            }
//
//            robotData.getCarosel().transferReceiveTimer();
//
//            return true;
        }
    }
    public Action patternFireWaitSpeed(double disp) {
        return new PatternFireWaitSpeed(disp);
    }
    public class PatternFire implements Action {

        public double disp;

        public PatternFire(double disp){
            this.disp = disp;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robotData.getCarosel().transferReceiveTimer();
            return robotData.getDriveTrain().executePatternFireAuto(disp);
//            if (robotData.getCarosel().getSortedFireCurrentShotCount() >= robotData.getCarosel().getMaxShots()) {
//                robotData.getCarosel().transferReceiveTimer();
//                robotData.getCarosel().resetSortedFireCurrentShotCount();
//                robotData.getDriveTrain().endSubMode();
//                return false;
//            }
//
//            switch (robotData.getCarosel().getCurrentSubModeQueue()) {
//                case ("cycle"):
//
//                    if (!robotData.getCarosel().isTransferCooldownActive()) {
//
//                        robotData.getCarosel().cycleSortedFire(robotData.getCarosel().getSortedFireCurrentShotCount());
//
//                        if (robotData.getCarosel().isCaroselInPlace()) {
//                            robotData.getCarosel().resetShotSuccess();
//                            robotData.getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
//                            robotData.getCarosel().activateTransferInProg();
//                        }
//                    }
//
//                    break;
//
//                case ("transfer"):
//
//                    if (robotData.getCarosel().getShotSuccessInstant() || robotData.getCarosel().isFailsafeSubmode()){
//                        robotData.getCarosel().endFailsafeSubmode();
//                        robotData.getCarosel().incrementSortedFireCurrentShotCount();
//                        robotData.getCarosel().activatePatternInProg();
//                        robotData.getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
//                        break;
//                    }
//
//                    boolean status;
//
//                    if (robotData.getTurret().isFarToggled()) status = Math.abs(robotData.getTurret().getTPSError(RobotConstantsV2.FAR_BALL_DISTANCE)) < robotData.getTurret().getTPS(RobotConstantsV2.FAR_BALL_DISTANCE) * RobotConstantsV2.SHOOTER_SPEED_THRESHOLD;
//                    else status = Math.abs(robotData.getTurret().getTPSError(RobotConstantsV2.CLOSE_BALL_DISTANCE)) < robotData.getTurret().getTPS(RobotConstantsV2.CLOSE_BALL_DISTANCE) * RobotConstantsV2.SHOOTER_SPEED_THRESHOLD;
//
//                    //Only Shoot up to speed
//                    if (status) robotData.getCarosel().subModeTransferStartTimer();
//
//                    robotData.getCarosel().checkShotSuccess();
//
//                    break;
//
//                default:
//                    break;
//            }
//
//            robotData.getCarosel().transferReceiveTimer();
//
//            return true;
        }
    }
    public Action patternFire(double disp) {
        return new PatternFire(disp);
    }
    public class RequestRapidFire implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            robotData.getDriveTrain().setCurrentSubMode(RobotConstantsV2.subModes[0]);
            robotData.getCarosel().activateCycleInProg();
            robotData.getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);

            return false;
        }
    }
    public Action requestRapidFire() {
        return new RequestRapidFire();
    }
    public class RapidFire implements Action{

        public double disp;

        public RapidFire(double disp){
            this.disp = disp;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robotData.getCarosel().transferReceiveTimer();
            return robotData.getDriveTrain().executeRapidFireAuto(disp);
//            if (robotData.getCarosel().getRapidFireCurrentShotCount() >= RobotConstantsV2.RAPID_FIRE_MAX_SHOTS) {
//                robotData.getCarosel().transferReceiveTimer();
//                robotData.getCarosel().resetRapidFireCurrentShotCount();
//                robotData.getDriveTrain().endSubMode();
//                return false;
//            }
//
//            switch (robotData.getCarosel().getCurrentSubModeQueue()){
//                case ("cycle"):
//
//                    if (!robotData.getCarosel().isTransferCooldownActive()){
//                        robotData.getCarosel().cycleRapidFire();
//
//                        if (robotData.getCarosel().isCaroselInPlace()){
//                            robotData.getCarosel().resetShotSuccess();
//                            robotData.getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
//                            robotData.getCarosel().activateTransferInProg();
//                        }
//                    }
//
//                    break;
//
//                case ("transfer"):
//
//                    if (!robotData.getCarosel().detectedArtifact() || robotData.getCarosel().getShotSuccessInstant() || robotData.getCarosel().isFailsafeSubmode()){
//                        robotData.getCarosel().endFailsafeSubmode();
//                        robotData.getCarosel().incrementRapidFireCurrentShotCount();
//                        robotData.getCarosel().activateCycleInProg();
//                        robotData.getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
//                        break;
//                    }
//
//                    robotData.getCarosel().subModeTransferStartTimer(); //Simplified
//                    robotData.getCarosel().checkShotSuccess();
//
//                    break;
//
//                default:
//                    break;
//
//            }
//
//            return true;
        }
    }
    public Action rapidFire(double disp) {
        return new RapidFire(disp);
    }
    public Action requestArtifactShots (boolean isPatternEnabled){
        if (isPatternEnabled) return new RequestPatternFire();
        else return new RequestRapidFire();
    }
    public Action shootArtifacts(double disp, boolean isPatternEnabled){
        if (isPatternEnabled) return new PatternFire(disp);
        else return new RapidFire(disp);
    }
    public class TurretPID implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            robotData.getTurret().aimBall(displacement);
            return loopingActive;
        }
    }
    public Action turretPID() {
        return new TurretPID();
    }
    public class TelemetryAuto implements Action{

        LimeLightVision limeLightVision;

        public TelemetryAuto(LimeLightVision limeLightVision){
            this.limeLightVision = limeLightVision;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            robotData.getTelemetry().addData("Starting Side", getRobotData().getStartingSide());
            robotData.getTelemetry().addData("Starting Position", getRobotData().getStartingPosition());
            robotData.getTelemetry().addData("Starting Color: ", limeLightVision.getAlliance());
            robotData.getTelemetry().addData("In Place: ", robotData.getCarosel().isCaroselInPlace());
            robotData.getTelemetry().addData("In Place Intake: ", robotData.getCarosel().isCaroselInPlaceIntake());

            robotData.getCarosel().telemetryCarosel();
            robotData.getTelemetry().update();

            return loopingActive;
        }
    }
    public Action telemetryAuto(LimeLightVision limeLightVision){
        return new TelemetryAuto(limeLightVision);
    }
    public class IndicatorUpdate implements Action{

        private LimeLightVision limeLight;

        public IndicatorUpdate(LimeLightVision limelight){
            this.limeLight = limelight;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            getRobotData().getCarosel().updateIndicators("manual",0);
            return loopingActive;
        }
    }
    public Action indicatorsUpdate(LimeLightVision limelight) {
        return new IndicatorUpdate(limelight);
    }
    public class WaitForTurret implements Action{

        private boolean active;

        public WaitForTurret(boolean active){
            this.active = active;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if (!active) return false;

            return !robotData.getTurret().isUpToSpeed(displacement);
        }
    }
    public Action waitForTurret(boolean active) {
        return new WaitForTurret(active);
    }
    public TrajectoryActionBuilder instantPath(Pose2d spawn) {
        return getDrive().actionBuilder(spawn);
    }

    public class CyclePattern implements Action{ //Creates Class, so each class is represented as an Action

        private int num;
        public CyclePattern(int num){
            this.num = num;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //Commands & Methods Go Here

            robotData.getCarosel().cyclePattern(num);

            return !robotData.getCarosel().isCaroselInPlace();
        }
    }
    public Action cyclePattern(int num) { //Creates method to return the action as an object
        return new CyclePattern(num); //Constructor
    }
    public class LocateAprilTag implements Action{ //Creates Class, so each class is represented as an Action

        private LimeLightVision limeLightVision;
        //private ElapsedTime failsafe;
        public LocateAprilTag(LimeLightVision limeLightVision){
            this.limeLightVision = limeLightVision;
            //failsafe = new ElapsedTime();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //Commands & Methods Go Here

//            if (!LimeLightVision.isFoundMotif){
                limeLightVision.updateMotifCode();
                robotData.getCarosel().updatePattern(LimeLightVision.motifCode);
//            }
//
//            //else if (failsafe.milliseconds() > RobotConstantsV2.LIMELIGHT_AUTO_FAILSAFE) return false;
//
//            else{
//                return false;
//            }

            return loopingActive;
        }
    }
    public Action locateAprilTag(LimeLightVision limeLightVision) { //Creates method to return the action as an object
        return new LocateAprilTag(limeLightVision); //Constructor
    }
    public class UpdateTelemetry implements Action{ //Creates Class, so each class is represented as an Action

        private Telemetry telemetry;
        public UpdateTelemetry(Telemetry telemetry){
            this.telemetry = telemetry;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //Commands & Methods Go Here

            robotData.updateTelemetry(telemetry);
            return loopingActive;
        }
    }
    public Action updateTelemetry(Telemetry telemetry) { //Creates method to return the action as an object
        return new UpdateTelemetry(telemetry); //Constructor
    }

    /** Base Methods */
    public void initPatternAuto(String[] pattern){
        robotData.getCarosel().updatePattern(pattern);
        robotData.getCarosel().setInventoryAuto();
    }
    public void doNothing(){}
    private double getPinpointHeadingError(double heading){
        return Math.abs(heading - intendedHeading);
    }
    public boolean isPinpointHeadingCorrect(double heading){
        if (heading < 0) heading += 360;
        return getPinpointHeadingError(heading) < RobotConstantsV2.PINPOINT_TOLERENCE * intendedHeading;
    }
    public void setIntendedHeading(double heading){
        this.intendedHeading = heading;
    }
    public void switchOpenGateActive(){
        if (openGateActive){
            openGateActive = false;
        }
        else{
            openGateActive = true;
        }
    }
    public boolean isOpenGateActive(){
        return openGateActive;
    }
    public void resetDriveConstants(){
        MecanumDrive.PARAMS.maxWheelVel = RobotConstantsV2.MAX_VEL_DEFAULT;
        MecanumDrive.PARAMS.minProfileAccel = RobotConstantsV2.MIN_ACCEL_DEFAULT;
        MecanumDrive.PARAMS.maxProfileAccel = RobotConstantsV2.MAX_ACCEL_DEFAULT;
    }

}
