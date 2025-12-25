package org.firstinspires.ftc.teamcode.RobotV2.ClassData;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

//Holds the data for RoadRunner in Autonomous and TeleOp, same robot object should be used
//Can create methods for TeleOp in RobotData and then call them in RoadRunner Data classes
public class RoadRunnerDataV2{

    private Pose2d beginPose;
    private MecanumDrive drive;
    private RobotDataV2 robotData;
    private ArrayList<Action> trajectoryBuilt;
    private FtcDashboard dash;
    private List<Action> teleOpActions;
    public static Pose2d lastAutoPosition = null; //TODO maybe issue
    private boolean notAutoPositionStored;
    private Pose2d parkingSpotBlue;
    private Pose2d parkingSpotRed;
    private ElapsedTime failsafeTimer;
    private boolean isDone;
    private boolean isDoneInit;
    private boolean closeSideFoundAT;
    private Action currentTeleOpAction;

//    public RoadRunnerData(HardwareMap hardwareMap) {
//        robot = new RobotData(hardwareMap);
//    }

    //Note to Future Me: Can either create a new robot object for roadrunner actions or use a pre-existing one to stay consistent

    //----------------------------------------
    public RoadRunnerDataV2(RobotDataV2 robot){
        this.robotData = robot;
        isDone = false;
        isDoneInit = false;
        closeSideFoundAT = false;

        teleOpActions = new ArrayList<>();
        trajectoryBuilt = new ArrayList<Action>();
        beginPose = null;
        notAutoPositionStored = false;

        parkingSpotBlue = new Pose2d(36.7,32.5,0);
        parkingSpotRed= new Pose2d(36.7,-32.5,0);
        failsafeTimer = new ElapsedTime();

        currentTeleOpAction = null;
    }

    //----------------------------------------

    //Trajectories


    public void setCloseSideFoundAT(boolean found){
        closeSideFoundAT = found;
    }
    public boolean getIsTurretDone(){
        return isDone;
    }

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

    public RobotDataV2 getRobotData() {
        return robotData;
    }
    public void updateRobot(RobotDataV2 robot) {
        this.robotData = robot;
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

    //----------------------------------------

    //Dashboard

    public void createDashboard(){
        dash = FtcDashboard.getInstance();
    }

    public FtcDashboard getDashboard(){
        return dash;
    }
//    public void updateTelemetryPacket(){
//        packet = new TelemetryPacket();
//    }

    //----------------------------------------

    //Create paths




    public void setCurrentTeleOpAction(Action a){
        currentTeleOpAction = a;
    }

    public void endTeleOpAction(){
        currentTeleOpAction = null;
    }

    //TODO Attempted to run trajectory as just an action
    public Action getParkingTrajectory(String alliance){

        Action park;

        if (alliance.equals("blue")){

            park = getDrive().actionBuilder(RobotConstantsV2.blueCorner)
                    .setTangent(Math.toRadians(225))
                    .splineToConstantHeading(RobotConstantsV2.parkingBlue,Math.toRadians(270))
                    .build();
        }
        else{
            park = getDrive().actionBuilder(RobotConstantsV2.redCorner)
                    .setTangent(Math.toRadians(135))
                    .splineToConstantHeading(RobotConstantsV2.parkingRed,Math.toRadians(90))
                    .build();
        }

        return park;
    }
    public Action getAlignTrajectory(double limelightYaw){

        Action park = getDrive().actionBuilder(new Pose2d(0,0,0))
                .turn(Math.toRadians(limelightYaw))
                .build();

        return park;

    }

    public void runCurrentTeleOpAction(){

        if (currentTeleOpAction != null){
            if (!currentTeleOpAction.run(new TelemetryPacket())){
                currentTeleOpAction = null;
            }
        }

        else{
            //TODO run DT
        }

    }





    public void requestPark(String alliance){

        if (alliance.equals("blue")){
            setBeginPose(RobotConstantsV2.blueCorner);
            createDrive();
        }
        else{
            setBeginPose(RobotConstantsV2.redCorner);
            createDrive();
        }
    }
    public void driveToPark(String alliance){

        if (alliance.equals("blue")){
            drive.setDrivePowers(new PoseVelocity2d(RobotConstantsV2.parkingBlue, 0));
        }
        else{
            drive.setDrivePowers(new PoseVelocity2d(RobotConstantsV2.parkingRed, 0));
        }

        drive.updatePoseEstimate();

        Pose2d pose = drive.localizer.getPose();

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

    }


//    public SequentialAction getParkingSeqAction(Pose2d currentPos, String alliance){
//        return new SequentialAction(
//                getParkingTrajectory(currentPos, alliance)
//        );
//    }

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

    //Robot Actions

    public class IntakeOn implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //Commands & Methods Go Here

            robotData.getCarosel().forceIntakeOn();

            return false;
        }
    }
    public Action intakeOn() {
        return new IntakeOn();
    }

    public class IntakeOff implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //Commands & Methods Go Here

            robotData.getCarosel().forceIntakeOff();

            return false;
        }
    }
    public Action intakeOff() {
        return new IntakeOff();
    }

    public Action quickUpdateInventory(){
        return new InstantAction(()->robotData.getCarosel().updateInventory());
    }

    public class CheckAutoIntake implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            robotData.getCarosel().autoIntakeCycle();
            robotData.getCarosel().receiveAutoCycleStatus();

            //robotData.getCarosel().isAutoIntakeCooldownActive()

            if (failsafeTimer.milliseconds() > RobotConstantsV2.AUTO_FAILSAFE_TIMER){
                return false;
            }

             //&& failsafeTimer.milliseconds() < RobotConstantsV2.AUTO_FAILSAFE_TIMER
            return (robotData.getCarosel().isEmptySpot() || robotData.getCarosel().isAutoIntakeCooldownActive());
        }
    }
    public Action checkAutoIntake() {
        return new CheckAutoIntake();
    }

    public class StartFailsafeTimer implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            failsafeTimer.reset();

           return false;
        }
    }
    public Action startFailsafeTimer() {
        return new StartFailsafeTimer();
    }

    public class PatternFire implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if (robotData.getCarosel().getSortedFireCurrentShotCount() >= robotData.getCarosel().getMaxShots()){
                robotData.getCarosel().transferReceiveTimer();
                robotData.getCarosel().resetSortedFireCurrentShotCount();
                return false;
            }

            switch (robotData.getCarosel().getCurrentSubModeQueue()){
                case ("cycle"):

                    if (!robotData.getCarosel().isTransferCooldownActive()){

                        robotData.getCarosel().cycleSortedFire(robotData.getCarosel().getSortedFireCurrentShotCount());

                        if (robotData.getCarosel().isCaroselInPlace()){
                            robotData.getCarosel().resetTransferStat();
                            robotData.getCarosel().resetShotSuccess();
                            robotData.getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
                            robotData.getCarosel().activateTransferInProg();
                        }

                    }

                    break;

                case ("transfer"):

                    boolean status;

                    if (robotData.getTurret().isFarToggled()){
                        status = Math.abs(robotData.getTurret().getTPSError(RobotConstantsV2.FAR_BALL_DISTANCE)) < robotData.getTurret().getTPS(RobotConstantsV2.FAR_BALL_DISTANCE) * RobotConstantsV2.SHOOTER_SPEED_THRESHOLD;

                    }


                    else{
                        status = Math.abs(robotData.getTurret().getTPSError(RobotConstantsV2.CLOSE_BALL_DISTANCE)) < robotData.getTurret().getTPS(RobotConstantsV2.CLOSE_BALL_DISTANCE) * RobotConstantsV2.SHOOTER_SPEED_THRESHOLD;
                    }

                    if (status){
                        robotData.getCarosel().startTransferCooldown();
                        robotData.getCarosel().cycleTransferStartTimer();
                        robotData.getCarosel().checkShotSuccess();

                    }
                    if (robotData.getCarosel().getShotSuccessInstant() || robotData.getCarosel().isFailsafeSubmode()){
                        robotData.getCarosel().endFailsafeSubmode();
                        robotData.getCarosel().incrementSortedFireCurrentShotCount();
                        robotData.getCarosel().activatePatternInProg();
                        robotData.getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
                        break;
                    }

                    break;

                default:
                    break;
            }

            robotData.getCarosel().transferReceiveTimer();

            //failsafeTimer.milliseconds() > RobotConstantsV2.AUTO_FAILSAFE_TIMER


            return true;
        }
    }

    public Action patternFire() {
        return new PatternFire();
    }

    public Action setTurretDone(boolean isDone){
        return new InstantAction(()-> this.isDone = isDone);
    }

    public class RequestPatternFire implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            robotData.getCarosel().activatePatternInProg();
            robotData.getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
            robotData.getCarosel().setMaxShotsPattern();

            return false;
        }
    }

    public Action requestPatternFire() {
        return new RequestPatternFire();
    }

    public class RequestRapidFire implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            robotData.getCarosel().activateCycleInProg();
            robotData.getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);

            return false;
        }
    }

    public Action requestRapidFire() {
        return new RequestRapidFire();
    }

    public class RapidFire implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if (robotData.getCarosel().getRapidFireCurrentShotCount() >= RobotConstantsV2.RAPID_FIRE_MAX_SHOTS) {
                robotData.getCarosel().resetRapidFireCurrentShotCount();
                return false;
            }

            switch (robotData.getCarosel().getCurrentSubModeQueue()){
                case ("cycle"):

                    if (!robotData.getCarosel().isTransferCooldownActive()){
                        robotData.getCarosel().cycleRapidFire();

                        if (robotData.getCarosel().isCaroselInPlace()){
                            robotData.getCarosel().resetTransferStat();
                            robotData.getCarosel().resetShotSuccess();
                            robotData.getCarosel().setSubModeQueue((RobotConstantsV2.subModeStages[1]));
                            robotData.getCarosel().activateTransferInProg();
                        }
                    }

                    break;

                case ("transfer"):
                    robotData.getCarosel().startTransferCooldown();
                    robotData.getCarosel().cycleTransferStartTimer();
                    robotData.getCarosel().checkShotSuccess();

                    if (robotData.getCarosel().getShotSuccessInstant() || robotData.getCarosel().isFailsafeSubmode()){
                        robotData.getCarosel().endFailsafeSubmode();
                        robotData.getCarosel().incrementRapidFireCurrentShotCount();
                        robotData.getCarosel().activateCycleInProg();
                        robotData.getCarosel().setSubModeQueue(RobotConstantsV2.subModeStages[0]);
                        break;
                    }

                    break;

                default:
                    break;

            }

            return true;
        }
    }

    public Action rapidFire() {
        return new RapidFire();
    }



    public class TurretPIDOn implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if (robotData.getTurret().isFarToggled()){
                robotData.getTurret().aimBall(RobotConstantsV2.FAR_BALL_DISTANCE);
            }

            else{
                robotData.getTurret().aimBallManual(robotData.getTurret().isFarToggled(),RobotConstantsV2.CLOSE_BALL_DISTANCE);
            }

            //robotData.getTurret().getTPSError(RobotConstantsV2.FAR_BALL_DISTANCE) < robotData.getTurret().getTPS(RobotConstantsV2.FAR_TPS) * RobotConstantsV2.SHOOTER_MAX_SPEED_THRESHOLD
            return !isDone;
        }
    }

    public Action turretPIDOn() {
        return new TurretPIDOn();
    }

    public class TelemetryAuto implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            robotData.getCarosel().telemetryCarosel();
            robotData.getTelemetry().update();

            return !isDoneInit;
        }
    }

    public Action setIsDoneInit(boolean done){
        return new InstantAction(() -> isDoneInit = done);
    }

    public Action telemetryAuto(){
        return new TelemetryAuto();
    }

    public Action updateTelemetry(){
        return new InstantAction(() ->robotData.getTelemetry().update());
    }

    public class IndicatorUpdate implements Action{

        private LimeLightVision limeLight;

        public IndicatorUpdate(LimeLightVision limelight){
            this.limeLight = limelight;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            getRobotData().getCarosel().updateIndicators("manual",RobotConstantsV2.FAR_BALL_DISTANCE,limeLight);
            return !isDoneInit;
        }
    }

    public Action indicatorsUpdate(LimeLightVision limelight) {
        return new IndicatorUpdate(limelight);
    }
    public class TurretPIDSetUp implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            boolean status;

            if (robotData.getTurret().isFarToggled()){
                robotData.getTurret().aimBall(RobotConstantsV2.FAR_BALL_DISTANCE);

                status = Math.abs(robotData.getTurret().getTPSError(RobotConstantsV2.FAR_BALL_DISTANCE)) > robotData.getTurret().getTPS(RobotConstantsV2.FAR_BALL_DISTANCE) * RobotConstantsV2.SHOOTER_SPEED_THRESHOLD;

            }

            else{
                robotData.getTurret().aimBallManual(robotData.getTurret().isFarToggled(),RobotConstantsV2.CLOSE_BALL_DISTANCE);
                status = Math.abs(robotData.getTurret().getTPSError(RobotConstantsV2.CLOSE_BALL_DISTANCE)) > robotData.getTurret().getTPS(RobotConstantsV2.CLOSE_BALL_DISTANCE) * RobotConstantsV2.SHOOTER_SPEED_THRESHOLD;
            }

            if (!status){
                isDoneInit = true;
                closeSideFoundAT = true;
            }

            return status;
        }
    }

    public Action setCloseSideFoundATHere(boolean AT){
        return new InstantAction(() -> closeSideFoundAT = AT);
    }

    public Action updatePattern(String[] pattern){
        return new InstantAction(() -> robotData.getCarosel().updatePattern(pattern));
    }

    public Action turretPIDSetUp() {
        return new TurretPIDSetUp();
    }

    public class KillTurret implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            robotData.getTurret().killShooter();

            return false;
        }
    }

    public Action killTurret() {
        return new KillTurret();
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

    public Action switchTransfer() { //Creates method to return the action as an object
        return new InstantAction(() -> robotData.getCarosel().toggleTransfer()); //Constructor
    }
    public Action forceTransferDown() { //Creates method to return the action as an object
        return new InstantAction(() -> robotData.getCarosel().forceTransferDown()); //Constructor
    }


    public void setCloseSidFoundAT(boolean AT){
        closeSideFoundAT = AT;
    }

    public class LocateAprilTag implements Action{ //Creates Class, so each class is represented as an Action

        private LimeLightVision limeLightVision;
        public LocateAprilTag(LimeLightVision limeLightVision){
            this.limeLightVision = limeLightVision;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //Commands & Methods Go Here

            if (!LimeLightVision.isFoundMotif){
                limeLightVision.updateMotifCode();
                robotData.getCarosel().updatePattern(LimeLightVision.motifCode);
            }

            else{
                closeSideFoundAT = true;
            }

            return !closeSideFoundAT;
        }
    }
    public Action locateAprilTag(LimeLightVision limeLightVision) { //Creates method to return the action as an object
        return new LocateAprilTag(limeLightVision); //Constructor
    }

    public Action updateCode(LimeLightVision limeLightVision){
        return new InstantAction(limeLightVision::updateMotifCode);
    }

    public Action updatePattern(){
        return new InstantAction(()-> robotData.getCarosel().updatePattern(LimeLightVision.motifCode));
    }

    public SequentialAction getSequentialConcept(){

        SequentialAction concept = new SequentialAction(
                //Other Method
                new SleepAction(10)
                //Other Method
        );

        return concept;

    }

    public void initPatternAuto(String[] pattern){
        robotData.getCarosel().updatePattern(pattern);
        robotData.getCarosel().setInventoryAuto();
    }

    public void updateRobotData(RobotDataV2 robotData){
        this.robotData = robotData;
    }

}
