//package org.firstinspires.ftc.teamcode.notcompetition.AUTO;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.utils.ArmTPD;
//import org.firstinspires.ftc.teamcode.utils.ExtenderPD;
//import org.firstinspires.ftc.teamcode.utils.ServoPose;
//import org.firstinspires.ftc.teamcode.utils.ServoPoseFollower;
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
//import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
//
//import java.util.ArrayList;
//import java.util.Arrays;
//import java.util.List;
//import java.util.Queue;
//import java.util.LinkedList;
//
//@Autonomous(name = "3Clips", group = "Autonomous")
//public class OCMoreClips extends OpMode{
//
//    private enum AutoState{
//        INITIALIZE, FIRST_PATH, SECOND_PATH, THIRD_PATH, FOURTH_PATH, FIFTH_PATH, SIXTH_PATH, SEVENTH_PATH, EIGHTH_PATH, NINTH_PATH, CLIP_DETECT_POSE, CLIP_UP_POSE, CLIP_RELEASE_POSE, PICK_POSE, PUT_POSE, ARM_UP, ARM_DOWN, EXTEND1, EXTEND2, EXTEND3,  PARK_PATH, COMPLETE;
//    }
//
//    private AutoState currentState = AutoState.INITIALIZE;
//    private final Timer opmodeTimer = new Timer();
//    private Follower follower;
//    private ServoPoseFollower servoPoseFollower;
//    private ArmTPD armT;
//    private ExtenderPD extender;
//    private Queue<AutoState> stateQueue = new LinkedList<>();
//
//    private PathChain firstPath, secondPath, thirdPath, fourthPath, fifthPath, sixthPath, seventhPath, eighthPath, ninthPath, parkPath;
//    public static double armT_kD = 0.00001, armT_kP = 0.001;
//    public static double extender_kD = 0.00001, extender_kP = 0.0005;
//
//    private static final Pose START_POSE = new Pose(9.00, 58.00, Math.toRadians(180));
//
//    @Override
//    public void init(){
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(START_POSE);
//
//        extender = new ExtenderPD(hardwareMap);
//        armT = new ArmTPD(hardwareMap);
//
//        defineInitialServoPoses(hardwareMap);
//        servoPoseFollower.start();
//
//        extender.resetEncoder();
//        armT.resetEncoder();
//
//        telemetry.addData("Init Status", "Initialized with starting pose and servo positions.");
//        telemetry.update();
//    }
//
//    @Override
//    public void start(){
//        firstPath = buildFirstPath();
//        secondPath = secondPath();
//        thirdPath = thirdPath();
//        fourthPath = fourthPath();
//        fifthPath = fifthPath();
//        sixthPath = sixthPath();
//        seventhPath = sevensPath();
//        eighthPath = eighthPath();
//        ninthPath = ninethPath();
//        parkPath = parkPath();
//
//        stateQueue.add(AutoState.ARM_UP);
//        stateQueue.add(AutoState.FIRST_PATH);
//        stateQueue.add(AutoState.ARM_DOWN);
//        stateQueue.add(AutoState.CLIP_RELEASE_POSE);
//        stateQueue.add(AutoState.SECOND_PATH);
//        stateQueue.add(AutoState.EXTEND1);
//        stateQueue.add(AutoState.PICK_POSE);
//        stateQueue.add(AutoState.THIRD_PATH);
//        stateQueue.add(AutoState.PUT_POSE);
//        stateQueue.add(AutoState.FOURTH_PATH);
//        stateQueue.add(AutoState.EXTEND2);
//        stateQueue.add(AutoState.PICK_POSE);
//        stateQueue.add(AutoState.FIFTH_PATH);
//        stateQueue.add(AutoState.PUT_POSE);
//        stateQueue.add(AutoState.SIXTH_PATH);
//        stateQueue.add(AutoState.CLIP_DETECT_POSE);
//        stateQueue.add(AutoState.CLIP_UP_POSE);
//        stateQueue.add(AutoState.ARM_UP);
//        stateQueue.add(AutoState.SEVENTH_PATH);
//        stateQueue.add(AutoState.ARM_DOWN);
//        stateQueue.add(AutoState.CLIP_RELEASE_POSE);
//        stateQueue.add(AutoState.EIGHTH_PATH);
//        stateQueue.add(AutoState.CLIP_DETECT_POSE);
//        stateQueue.add(AutoState.CLIP_UP_POSE);
//        stateQueue.add(AutoState.ARM_UP);
//        stateQueue.add(AutoState.NINTH_PATH);
//        stateQueue.add(AutoState.ARM_DOWN);
//        stateQueue.add(AutoState.CLIP_RELEASE_POSE);
//        stateQueue.add(AutoState.PARK_PATH);
//        stateQueue.add(AutoState.COMPLETE);
//
//        setNextState();
//    }
//
//    @Override
//    public void loop(){
//        armT.setPID(armT_kP, 0, armT_kD);
//        extender.setPID(extender_kP, 0, extender_kD);
//        follower.update();
//        servoPoseFollower.update();
//
//        if (currentState == null && !stateQueue.isEmpty()) {
//            setNextState();
//        }
//
//        if(currentState != null){
//            switch (currentState){
//                // Grouped follower-based states
//                case FIRST_PATH: case SECOND_PATH: case THIRD_PATH:
//                case FOURTH_PATH: case FIFTH_PATH: case SIXTH_PATH:
//                case SEVENTH_PATH: case EIGHTH_PATH: case NINTH_PATH:
//                case PARK_PATH:
//                    if (follower.isCloseEnoughToEnd()) {
//                        setNextState();
//                    }
//                    break;
//
//                // Grouped Arm-Based States
//                case ARM_UP: case ARM_DOWN:
//                    if(armT.isAtTarget()) setNextState();
//                    break;
//
//                // Grouped Extender-Based States
//                case EXTEND1: case EXTEND2: case EXTEND3:
//                    if(extender.isAtTarget()) setNextState();
//                    break;
//
//                // Grouped servo-based states
//                case CLIP_RELEASE_POSE: case CLIP_DETECT_POSE: case CLIP_UP_POSE:
//                case PICK_POSE: case PUT_POSE:
//                    if (servoPoseFollower.isComplete()) {
//                        setNextState();
//                    }
//                    break;
//
//                case COMPLETE:
//                    telemetry.addData("Status", "Autonomous Complete");
//                    break;
//
//        }
//
//        }
//
//        telemetry.addData("Current State", currentState);
//        telemetry.addData("Follower X", follower.getPose().getX());
//        telemetry.addData("Follower Y", follower.getPose().getY());
//        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
//        telemetry.addData("Timer", opmodeTimer.getElapsedTime());
//        telemetry.update();
//    }
//
//    @Override
//    public void stop(){
//        telemetry.addData("Status", "Autonomous Stopped");
//        telemetry.update();
//    }
//
//    private void setNextState() {
//        if (!stateQueue.isEmpty()) {
//            currentState = stateQueue.poll();
//            opmodeTimer.resetTimer();
//            applyStateAction(currentState);
//        } else {
//            currentState = null;
//        }
//    }
//
//    private void applyStateAction(AutoState newState){
//        currentState = newState;
//        opmodeTimer.resetTimer();
//        switch(newState){
//            case FIRST_PATH: follower.followPath(firstPath); break;
//            case SECOND_PATH: follower.followPath(secondPath); break;
//            case THIRD_PATH: follower.followPath(thirdPath); break;
//            case FOURTH_PATH: follower.followPath(fourthPath); break;
//            case FIFTH_PATH: follower.followPath(fifthPath); break;
//            case SIXTH_PATH: follower.followPath(sixthPath); break;
//            case SEVENTH_PATH: follower.followPath(seventhPath); break;
//            case EIGHTH_PATH: follower.followPath(eighthPath); break;
//            case NINTH_PATH: follower.followPath(ninthPath); break;
//            case PARK_PATH: follower.followPath(parkPath); break;
//            case ARM_UP: armT.setTargetPosition(4000); break;
//            case ARM_DOWN: armT.setTargetPosition(2000); break;
//            case EXTEND1: extender.setTargetPosition(-1000); break;
//            case EXTEND2: extender.setTargetPosition(-1200); break;
//            case EXTEND3: extender.setTargetPosition(-1500); break;
//            case CLIP_DETECT_POSE: armT.setTargetPosition(0); defineClipDetectServoPoses(hardwareMap); servoPoseFollower.start(); break;
//            case CLIP_UP_POSE: armT.setTargetPosition(450); defineClipUpServoPoses(hardwareMap); servoPoseFollower.start(); break;
//            case CLIP_RELEASE_POSE: defineClipReleaseServoPoses(hardwareMap); servoPoseFollower.start(); break;
//            case PICK_POSE: definePickServoPoses(hardwareMap); servoPoseFollower.start(); break;
//            case PUT_POSE: definePutServoPoses(hardwareMap); servoPoseFollower.start(); break;
//        }
//    }
//    //===================================================== PATHS PART ==================================================================
//    private PathChain buildFirstPath(){
//        return follower.pathBuilder()
//                .addPath(
//                        // Line 1
//                        new BezierLine(
//                                new Point(9.000, 58.000, Point.CARTESIAN),
//                                new Point(41.272, 67.180, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//    }
//    private PathChain secondPath(){
//        return follower.pathBuilder()
//                .addPath(
//                        // Line 2
//                        new BezierLine(
//                                new Point(41.272, 67.180, Point.CARTESIAN),
//                                new Point(28.921, 32.234, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-35))
//                .build();
//    }
//    private PathChain thirdPath(){
//        return follower.pathBuilder()
//                .addPath(
//                        // Line 3
//                        new BezierLine(
//                                new Point(28.921, 32.234, Point.CARTESIAN),
//                                new Point(28.921, 32.234, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(-35), Math.toRadians(-150))
//                .build();
//    }
//    private PathChain fourthPath(){
//        return follower.pathBuilder()
//                .addPath(
//                        // Line 4
//                        new BezierLine(
//                                new Point(28.921, 32.234, Point.CARTESIAN),
//                                new Point(28.921, 32.234, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(-150), Math.toRadians(-40))
//                .build();
//    }
//    private PathChain fifthPath(){
//        return follower.pathBuilder()
//                .addPath(
//                        // Line 5
//                        new BezierLine(
//                                new Point(28.921, 32.234, Point.CARTESIAN),
//                                new Point(28.921, 32.234, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(-150))
//                .build();
//    }
//    private PathChain sixthPath(){
//        return follower.pathBuilder()
//                .addPath(
//                        // Line 6
//                        new BezierLine(
//                                new Point(28.921, 32.234, Point.CARTESIAN),
//                                new Point(9.000, 24.703, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(-150), Math.toRadians(180))
//                .build();
//    }
//    private PathChain sevensPath(){
//        return follower.pathBuilder()
//                .addPath(
//                        // Line 7
//                        new BezierLine(
//                                new Point(9.000, 24.703, Point.CARTESIAN),
//                                new Point(41.272, 71.397, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//    }
//    private PathChain eighthPath(){
//        return follower.pathBuilder()
//                .addPath(
//                        // Line 8
//                        new BezierLine(
//                                new Point(41.272, 71.397, Point.CARTESIAN),
//                                new Point(9.000, 24.703, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//    }
//    private PathChain ninethPath(){
//        return follower.pathBuilder()
//                .addPath(
//                        // Line 9
//                        new BezierLine(
//                                new Point(9.000, 24.703, Point.CARTESIAN),
//                                new Point(41.272, 75.314, Point.CARTESIAN)
//                        )
//                )
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//    }
//
//    private PathChain parkPath(){
//        return follower.pathBuilder()
//                .addPath(
//                        // Line 10
//                        new BezierLine(
//                                new Point(41.272, 75.314, Point.CARTESIAN),
//                                new Point(9.640, 10.243, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
//                .build();
//    }
//    //======================================================= SERVO SETTING PART =======================================================================
//    private void defineInitialServoPoses(HardwareMap hardwareMap) {
//        List<ServoPose> initialPoses = Arrays.asList(
//                new ServoPose(0.65, 0.35, 0.85, 0.5, 1,100)
//        );
//        servoPoseFollower = new ServoPoseFollower(hardwareMap, initialPoses);
//    }
//
//    private void defineClipDetectServoPoses(HardwareMap hardwareMap) {
//        List<ServoPose> detectPoses = Arrays.asList(
//                new ServoPose(0.35, 0.35, 0.85, 0.6, 1,100)
//        );
//        servoPoseFollower = new ServoPoseFollower(hardwareMap, detectPoses);
//    }
//
//    private void defineClipUpServoPoses(HardwareMap hardwareMap) {
//        List<ServoPose> upPoses = Arrays.asList(
//                new ServoPose(0.65, 0.35, 0.85, 0.5, 1,300)
//        );
//        servoPoseFollower = new ServoPoseFollower(hardwareMap, upPoses);
//    }
//
//    private void defineClipReleaseServoPoses(HardwareMap hardwareMap) {
//        List<ServoPose> releasePoses = Arrays.asList(
//                new ServoPose(0.35, 0.35, 0.85, 0.5, 1,300)
//        );
//        servoPoseFollower = new ServoPoseFollower(hardwareMap, releasePoses);
//    }
//
//    private void definePickServoPoses(HardwareMap hardwareMap) {
//        List<ServoPose> pickPoses = Arrays.asList(
//                new ServoPose(0.35, 0.35, 0.3, 1, 0.4,500),
//                new ServoPose(0.35, 0.65, 0.3, 1, 0,300),
//                new ServoPose(0.35, 0.65, 0.3, 1, 0.4,300)
//        );
//        servoPoseFollower = new ServoPoseFollower(hardwareMap, pickPoses);
//    }
//
//    private void definePutServoPoses(HardwareMap hardwareMap) {
//        List<ServoPose> putPoses = Arrays.asList(
//                new ServoPose(0.35, 0.65, 0.85, 1, 0.4,300),
//                new ServoPose(0.35, 0.35, 0.85, 1, 0, 500)
//        );
//        servoPoseFollower = new ServoPoseFollower(hardwareMap, putPoses);
//    }
//}
