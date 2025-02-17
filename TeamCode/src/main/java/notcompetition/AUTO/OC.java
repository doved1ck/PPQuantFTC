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
//
//@Autonomous(name = "1Clip", group = "Autonomous")
//public class OC extends OpMode{
//
//    private enum AutoState{
//        INITIALIZE, FIRST_PATH, CLIP_DETECT_POSE, CLIP_UP_POSE, FIRST_POSE, SECOND_POSE, THIRD_POSE, ARM_UP, ARM_DOWN, PARK_PATH, COMPLETE;
//    }
//
//    private AutoState currentState = AutoState.INITIALIZE;
//    private final Timer opmodeTimer = new Timer();
//    private Follower follower;
//    private ServoPoseFollower servoPoseFollower;
//    private ArmTPD armT;
//
//    private PathChain firstPath, parkPath;
//    public static double armT_kD = 0.00001, armT_kP = 0.001;
//
//    private static final Pose START_POSE = new Pose(9.00, 58.00, Math.toRadians(180));
//
//    @Override
//    public void init(){
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(START_POSE);
//
//        armT = new ArmTPD(hardwareMap);
//
//        defineInitialServoPoses(hardwareMap);
//        servoPoseFollower.start();
//
//        armT.resetEncoder();
//
//        telemetry.addData("Init Status", "Initialized with starting pose and servo positions.");
//        telemetry.update();
//    }
//
//    @Override
//    public void start(){
//        firstPath = buildFirstPath();
//        parkPath = parkPath();
//
//        setState(AutoState.FIRST_POSE);
//    }
//
//    @Override
//    public void loop(){
//        armT.setPID(armT_kP, 0, armT_kD);
//        follower.update();
//        servoPoseFollower.update();
//        switch (currentState){
//            case FIRST_POSE:
//                if(servoPoseFollower.isComplete()) setState(AutoState.ARM_UP);
//                break;
//            case ARM_UP:
//                if(armT.isAtTarget()) setState(AutoState.FIRST_PATH);
//                break;
//            case FIRST_PATH:
//                if(follower.isCloseEnoughToEnd()) setState(AutoState.ARM_DOWN);
//                break;
//            case ARM_DOWN:
//                if(armT.isAtTarget()) setState(AutoState.SECOND_POSE);
//                break;
//            case SECOND_POSE:
//                if(servoPoseFollower.isComplete()) setState(AutoState.PARK_PATH);
//                break;
//            case PARK_PATH:
//                if(follower.isCloseEnoughToEnd()) setState(AutoState.COMPLETE);
//                break;
//            case COMPLETE:
//                telemetry.addData("Status", "Autonomous Complete");
//                break;
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
//    private void setState(AutoState newState){
//        currentState = newState;
//        opmodeTimer.resetTimer();
//        switch(newState){
//            case FIRST_PATH: follower.followPath(firstPath); break;
//            case PARK_PATH: follower.followPath(parkPath); break;
//            case FIRST_POSE: defineInitialServoPoses(hardwareMap); servoPoseFollower.start(); break;
//            case SECOND_POSE: defineSecondServoPoses(hardwareMap); servoPoseFollower.start(); break;
//            case ARM_UP: armT.setTargetPosition(4000); break;
//            case ARM_DOWN: armT.setTargetPosition(2000); break;
//            case CLIP_DETECT_POSE: armT.setTargetPosition(0); defineClipDetectServoPoses(hardwareMap); servoPoseFollower.start(); break;
//            case CLIP_UP_POSE: armT.setTargetPosition(450); defineClipUpServoPoses(hardwareMap); servoPoseFollower.start(); break;
//        }
//    }
////===================================================== PATHS PART ==================================================================
//private PathChain buildFirstPath(){
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
//
//    private PathChain parkPath(){
//        return follower.pathBuilder()
//                .addPath(
//                        // Line 2
//                        new BezierLine(
//                                new Point(41.272, 67.180, Point.CARTESIAN),
//                                new Point(9.000, 7.624, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
//                .build();
//    }
////======================================================= SERVO SETTING PART =======================================================================
//    private void defineInitialServoPoses(HardwareMap hardwareMap) {
//        List<ServoPose> initialPoses = Arrays.asList(
//                new ServoPose(0.65, 0.35, 0.85, 0.5, 1,100)
//        );
//        servoPoseFollower = new ServoPoseFollower(hardwareMap, initialPoses);
//    }
//
//    private void defineClipDetectServoPoses(HardwareMap hardwareMap) {
//        List<ServoPose> DetectPoses = Arrays.asList(
//                new ServoPose(0.35, 0.35, 0.85, 0.6, 1,100)
//        );
//        servoPoseFollower = new ServoPoseFollower(hardwareMap, DetectPoses);
//    }
//
//    private void defineClipUpServoPoses(HardwareMap hardwareMap) {
//        List<ServoPose> UpPoses = Arrays.asList(
//                new ServoPose(0.65, 0.35, 0.85, 0.5, 1,300)
//        );
//        servoPoseFollower = new ServoPoseFollower(hardwareMap, UpPoses);
//    }
//
//    private void defineSecondServoPoses(HardwareMap hardwareMap) {
//        List<ServoPose> secondPoses = Arrays.asList(
//                new ServoPose(0.35, 0.35, 0.85, 0.5, 1,300)
//        );
//        servoPoseFollower = new ServoPoseFollower(hardwareMap, secondPoses);
//    }
//
//}
