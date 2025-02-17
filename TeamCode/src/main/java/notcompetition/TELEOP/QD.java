package notcompetition.TELEOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

@Config
@TeleOp
public class QD extends LinearOpMode{
   public static int liftOutLimit = -2500, liftInLimit = 0;
   public static int extendOutLimit = 2500, extendInLimit = 0;

   public static double wristBPos = 0;
   public static int armTDownPos = 0, armTClipPos = 4000, armTHighBasketPos = 3000, armTTransferPos = 200;
   public static double armT_Kd = 0.00001, armT_Kp = 0.001, armT_Ki = 0.001;
//   public static double extender_Kd = 0.00001, extender_Kp = 0.0005, extender_Ki = 0.0005;
   public static double extender_Kd = 0.00001, extender_Kp = 0.002, extender_Ki = 0.005;
   public static double lift_kD = 0.0001, lift_kP = 0.001;
   public static double heading_Kp = 0.04, heading_kD = 0.002, heading_kI = 0;
   //at kP = 0.0004, kD = 0.00035, kI = 0 at 12.5V battery
    //at kP = 0.04 , kD = 0.002, kI = 0 at 13.4V battery
//   public static double extenderSpeed = 3000;
   public static double extenderSpeed = 1000;

   public static double turnSpeed = 180;

   public static double liftSpeed = 2200;

   @Override
   public void runOpMode() throws InterruptedException{
       Motor backLeftMotor = new Motor(hardwareMap, "leftBack");
       Motor backRightMotor = new Motor(hardwareMap, "rightBack");
       Motor frontLeftMotor = new Motor(hardwareMap, "leftFront");
       Motor frontRightMotor = new Motor(hardwareMap, "rightFront");
       Motor lift1 = new Motor(hardwareMap, "lift1");
       Motor lift2 = new Motor(hardwareMap, "lift2");
       Motor arm = new Motor(hardwareMap, "armT");
       Motor extender = new Motor(hardwareMap, "extendB");

       TouchSensor touchSensor = hardwareMap.touchSensor.get("touch");
       VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

       ServoImplEx armBServo = (ServoImplEx) hardwareMap.servo.get("armB");
       ServoImplEx clawTServo = (ServoImplEx) hardwareMap.servo.get("clawT");
       ServoImplEx clawBServo = (ServoImplEx) hardwareMap.servo.get("clawB");
       ServoImplEx wristBServo = (ServoImplEx) hardwareMap.servo.get("wristB");
       ServoImplEx wristTServo = (ServoImplEx) hardwareMap.servo.get("wristT");

       PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);

       armBServo.setPwmRange(pwmRange);
       clawTServo.setPwmRange(pwmRange);
       clawBServo.setPwmRange(pwmRange);
       wristBServo.setPwmRange(pwmRange);
       wristTServo.setPwmRange(pwmRange);

       frontLeftMotor.setInverted(true);
       backLeftMotor.setInverted(true);
       lift2.setInverted(true);

       PDController lift1PD = new PDController(0,0);
       PDController lift2PD = new PDController(0,0);
       PDController armPD = new PDController(0,0);
       PDController extenderPD = new PDController(0,0);
       PDController headingPD = new PDController(0,0);

       Follower follower = new Follower(hardwareMap);
       FtcDashboard dashboard = FtcDashboard.getInstance();

       MecanumDrive drive = new MecanumDrive(frontLeftMotor,frontRightMotor,backLeftMotor,backRightMotor);

       double liftTargetPos = 0;
       double armTargetPos = 0;
       double extenderTargetPos = 0;
       double headingTarget = 0;
       int wristBPositionState = 0;
       int clawTState = 0;

       //constants
       double armBPosition = 1;
       boolean clawBclosed = true;
       boolean transferStarted = false;
       boolean transferPiece = false;
       boolean clawTclosed = false;
       boolean armBbuttonPressed = false;
       boolean needReset = false;
       boolean modeEnabled = false;
       boolean modeButtonPressed = false;
       boolean clawTAnimating = false;
       boolean clawOpening = false;


       GamepadEx driver = new GamepadEx(gamepad1);
       GamepadEx tool = new GamepadEx(gamepad2);

       waitForStart();

       if (isStopRequested()) return;

       follower.setStartingPose(new Pose(0,0,0));
       extender.resetEncoder();
       lift1.resetEncoder();
       lift2.resetEncoder();
       arm.resetEncoder();

       //servo poses on init
       clawTServo.setPosition(0.35);
       clawBServo.setPosition(0.35);
       wristBServo.setPosition(0.85);
       wristTServo.setPosition(0.5);

       ElapsedTime runTimer = new ElapsedTime();
       ElapsedTime clawTimer = new ElapsedTime();
       ElapsedTime clawBTimer = new ElapsedTime();

       boolean delayFlag = false;
       boolean catchMode = false;


       while(opModeIsActive()){
           driver.readButtons();
           tool.readButtons();

           double deltaTime = ((double)runTimer.milliseconds() / 1000.0);
           runTimer.reset();

           lift1PD.setPID(lift_kP, 0, lift_kD);
           lift2PD.setPID(lift_kP, 0, lift_kD);
           armPD.setPID(armT_Kp, 0, armT_Kd);
           extenderPD.setPID(extender_Kp, 0, extender_Kd);
           headingPD.setPID(heading_Kp, 0, heading_kD);

           follower.update();
           double currentHeading = Math.toDegrees(follower.getPose().getHeading());
           telemetry.addData("Heading", currentHeading);
           telemetry.addData("Target Heading", headingTarget);

           double forwardInput = gamepad1.left_stick_y; // Remember, Y stick value is reversed
           double strafeInput = -gamepad1.left_stick_x;
           double turnInput = -gamepad1.right_stick_x;

           //=============================================== heading code ===============================================

           headingTarget += turnInput * turnSpeed * deltaTime;

           if(driver.wasJustPressed(GamepadKeys.Button.B)){
               if((int)headingTarget%90 == 0) headingTarget -= 90;
               else headingTarget = Math.floor(headingTarget / 90.0 ) * 90.0;
           }

           if(driver.wasJustPressed(GamepadKeys.Button.X)){
               if((int)headingTarget % 90 == 0) headingTarget += 90;
               else headingTarget = Math.ceil(headingTarget/90.0) * 90.0;
           }

           if(gamepad1.options){
               headingTarget = 0;
               follower.setStartingPose(new Pose(0,0,0));
           }


           double headingError = getHeadingError(currentHeading, headingTarget);
           double batteryVoltage = voltageSensor.getVoltage();
           double turnPower = -headingPD.calculate(headingError, 0);
           //TODO: decomment if needed
//           turnPower *= (12/ batteryVoltage);


//           drive.driveRobotCentric(strafeInput,forwardInput,turnPower);
           drive.driveFieldCentric(strafeInput, forwardInput, turnPower, currentHeading);

           telemetry.addData("Strafe input", strafeInput);
           telemetry.addData("forwardInput", forwardInput);
           telemetry.addData("Turn Power", turnPower);
           //============================================ lift code ==================================================

           double liftInput = 0;

           if(gamepad2.right_bumper){
                liftTargetPos = 0;
           }
           else if(gamepad2.left_bumper && !touchSensor.isPressed()){
               transferStarted = true;
               liftTargetPos = -2800;
           }
           else if(transferPiece && !transferStarted){
               if(lift1.getCurrentPosition() > -170){
                   lift1.stopMotor();
                   lift2.stopMotor();
               }
               else {
                   lift1.stopMotor();
                   lift2.stopMotor();
               }
               liftTargetPos = -700;
           }

          /* liftTargetPos -= liftInput * liftSpeed * deltaTime;*/
           if(liftTargetPos == 0 && !touchSensor.isPressed() && !needReset) {
               needReset = true;
               liftTargetPos = 2000;
           }
           else if(touchSensor.isPressed() && needReset){
               liftTargetPos = 0;
               needReset = false;
               lift1.stopAndResetEncoder();
               lift2.stopAndResetEncoder();
           }


           double lift1Pos = lift1.getCurrentPosition();
           double lift2Pos = lift2.getCurrentPosition();

           double lift1Power = lift1PD.calculate(lift1Pos, liftTargetPos);
           double lift2Power = lift2PD.calculate(lift2Pos, liftTargetPos);

           telemetry.addData("Lift1 Position", lift1.getCurrentPosition());
           telemetry.addData("Lift2 Position", lift2.getCurrentPosition());
           telemetry.addData("Lift1 Power", lift1Power);
           telemetry.addData("Lift2 Power", lift2Power);
           telemetry.addData("Lifts Target Position", liftTargetPos);

           lift1.set(lift1Power);
           lift2.set(lift2Power);

           // =======================================extender code=====================================

           double extenderInput = 0;
           if(gamepad2.right_trigger > 0.2) extenderInput += 1;
           if(gamepad2.left_trigger > 0.2) extenderInput -= 1;

           extenderTargetPos += extenderInput * extenderSpeed * deltaTime;
           extenderTargetPos = Math.max(-1500, Math.min(extenderTargetPos, 300));
           double extenderPos = extender.getCurrentPosition();
           extender.set(extenderPD.calculate(extenderPos, extenderTargetPos));

           if ((extenderTargetPos == 300 && extenderPos >= 300) || (extenderTargetPos == -1500 && extenderPos <= -1500)) {
               extender.set(0);
           } else {
               extender.set(extenderPD.calculate(extenderPos, extenderTargetPos));
           }

           telemetry.addData("Extender Target Pos", extenderTargetPos);
           telemetry.addData("Extender Current Pos", extenderPos);

           //==========================armT code==============================

           if (driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
               armTargetPos = (armTargetPos == armTDownPos) ? armTHighBasketPos : armTDownPos;
           }

           double armPos = arm.getCurrentPosition();
           arm.set(armPD.calculate(armPos, armTargetPos));

           telemetry.addData("Arm Target Pos", armTargetPos);
           telemetry.addData("Arm Current Pos", armPos);


           //==================================Servo settings=======================================
           //=====================================wristB============================================
           if(tool.wasJustPressed(GamepadKeys.Button.X)){

               // Increment position state, looping back to 0 when reaching 3
               wristBPositionState = (wristBPositionState + 1) % 4;

               // Set wristB position based on current state
               switch (wristBPositionState) {
                   case 0: // Starting position (0 degrees)
                       wristBServo.setPosition(0.5);  // Adjust this value for the starting position
                       break;
                   case 1: // 45 degrees left
                       wristBServo.setPosition(0.3);  // Adjust this value for 45 degrees to the left
                       break;
                   case 2: // 45 degrees right
                       wristBServo.setPosition(0.7);  // Adjust this value for 45 degrees to the right
                       break;
                   case 3: // 90 degrees right
                       wristBServo.setPosition(0.85);  // Adjust this value for 90 degrees to the right
                       break;
               }
           }

           telemetry.addData("wristB pos", wristBServo.getPosition());

           //================================================wristT==========================================

           if(tool.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) wristTServo.setPosition(1);
           else if(tool.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) wristTServo.setPosition(0);

           telemetry.addData("wristT Pos", wristTServo.getPosition());


           //================================================clawT=============================================

//           if (tool.isDown(GamepadKeys.Button.A)) {
//               transferPiece = false;
//               liftTargetPos = 0;
//           }
//
//           if (tool.wasJustReleased(GamepadKeys.Button.A)) {
//               clawTAnimating = true;
//               clawTState = 0;
//               clawTimer.reset();
//           }
//
//           if (clawTAnimating) {
//               switch (clawTState) {
//                   case 0:
//                       clawTclosed = !clawTclosed;  // Close claw
//                       clawTState++;
//                       clawTimer.reset();
//                       break;
//                   case 1:
//                       if (clawTimer.milliseconds() > 200) {
//                           clawTServo.setPosition(0.65);
//                           clawTState++;
//                           clawTimer.reset();
//                       }
//                       break;
//                   case 2:
//                       if (clawTimer.milliseconds() > 200) {
//                           clawBServo.setPosition(0.35);
//                           clawTState++;
//                           clawTimer.reset();
//                       }
//                       break;
//                   case 3:
//                       if (clawTimer.milliseconds() > 200) {
//                           liftTargetPos = -2800;
//                           clawTState++;
//                           clawTimer.reset();
//                       }
//                       break;
//                   case 4:
//                       if (clawTimer.milliseconds() > 1000) {
//                           armTargetPos = armTHighBasketPos;
//                           clawTState++;
//                           clawTimer.reset();
//                       }
//                       break;
//                   case 5:
//                       if(clawTimer.milliseconds() > 200){
//                           wristTServo.setPosition(1);
//                           clawTState++;
//                           clawTimer.reset();
//                       }
//                       break;
//               }
//           }
           if(driver.wasJustPressed(GamepadKeys.Button.Y)) clawTServo.setPosition(0.35);


           telemetry.addData("ClawT state", clawTState);
           telemetry.addData("ClawT Animation complete", clawTAnimating);
           telemetry.addData("transferPiece state", transferPiece);
           telemetry.addData("transferStarted state", transferStarted);
           telemetry.addData("clawT Pos", clawTServo.getPosition());

           //==========================================clawB====================================

           if (tool.wasJustPressed(GamepadKeys.Button.B)) {
               clawBclosed = !clawBclosed;
               clawBTimer.reset();
               if (!clawBclosed && armBPosition == 0.4 && !catchMode) {
                   catchMode = true;
               }else {
                   // Open the claw logic
                   clawBServo.setPosition(0.35);

               }

//               if (clawBclosed) {
//                   // If the claw is closing and armBPosition is 0.15, lower the arm
//                   if (armBPosition == 0.4) {
//                       armBPosition = 0;
//                       armBServo.setPosition(armBPosition);
//                       sleep(200);
//                   }
//
//                   clawBServo.setPosition(0.65); // Close claw
//                   sleep(200);
//
//                   if (armBPosition == 0) {
//                       armBPosition = 0.4;
//                       armBServo.setPosition(armBPosition);
//                   }
//               } else {
//                   // Open the claw
//                   clawBServo.setPosition(0.35);
//               }
           }
               if(catchMode) {
                   // If the armBPosition is 0.4, and the claw needs to close, lower the arm with a delay
                   if (armBPosition == 0.4 && !delayFlag) {
                       armBPosition = 0.0;
                       armBServo.setPosition(armBPosition);
                       delayFlag = true;  // Set the flag to prevent the action from triggering repeatedly
                   }

                   // If enough time has passed (200ms), close the claw
                   if (delayFlag && (clawBTimer.milliseconds() >= 200)) {
                       clawBServo.setPosition(0.7);  // Close the claw
                       delayFlag = false;  // Reset the flag after the action
                       armBPosition = 0.4;
                       armBServo.setPosition(armBPosition);
                       catchMode = false;
                   }
               }

           telemetry.addData("clawB Pos", clawBServo.getPosition());

           //==========================================armB=========================================
//           if (gamepad2.y && !armBbuttonPressed) {
//               transferStarted = false;
//               // Cycle through positions when button is pressed
//               if (armBPosition == 0.4) {
//                   armBPosition = 1; // Move to transfer position+
//                   if (!transferStarted) {
//                       transferPiece = true;
//                   }
//                   armTargetPos = 0;
//                   wristBServo.setPosition(0.85);
//                   wristTServo.setPosition(0.2);
//                   clawBServo.setPosition(0.65);
//                   clawTServo.setPosition(0.35);
//                   extenderTargetPos = 0;
//
//               } else {
//                   transferPiece = false;
//                   armBPosition = 0.4; // Move back to starting position
//               }
//               armBbuttonPressed = true;
//           }
//            else if (!gamepad2.y) {
//               // Reset button press state when button is released
//               armBbuttonPressed = false;
//           }
//           armBServo.setPosition(armBPosition);

           telemetry.addData("ArmB Pos", armBPosition);

           //=======================================================================clip code================================================================



           //======================================================================mode button=============================================================

           if(gamepad2.guide && !modeButtonPressed){
               modeEnabled = !modeEnabled;
               modeButtonPressed = true;
           }
           else if(!gamepad2.guide) modeButtonPressed = false;

           if(modeEnabled){
               transferPiece = false;
              if(tool.wasJustPressed(GamepadKeys.Button.Y)){
                  clawTServo.setPosition(0.35);
                  armTargetPos = 0;
                  wristTServo.setPosition(0.6);
              }
              if(tool.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                  wristTServo.setPosition(0.5);
                  armTargetPos = armTClipPos;
              }
              else if (tool.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                  armTargetPos = armTClipPos - 2000 ;
                  clawTimer.reset();
                  clawOpening = true;
              }
              if (clawOpening && clawTimer.milliseconds() > 300) {
                   clawTServo.setPosition(0.35);
                   clawOpening = false;
              }
              if(tool.wasJustPressed(GamepadKeys.Button.A)){
                  armTargetPos = 450;
                  clawTState = 0;
                  clawTAnimating = true;
                  clawTimer.reset();
              }
               if (clawTAnimating) {
                   switch (clawTState) {
                       case 0:
                           if (clawTimer.milliseconds() > 300) {
                               clawTServo.setPosition(0.7);
                               clawTState++;
                               clawTimer.reset();
                           }
                           break;
                       case 1:
                           clawTAnimating = false;
                           break;
                   }
               }
           }
           else{
               if (tool.isDown(GamepadKeys.Button.A)) {
                   transferPiece = false;
                   liftTargetPos = 0;
               }

               if (tool.wasJustReleased(GamepadKeys.Button.A)) {
                   clawTAnimating = true;
                   clawTState = 0;
                   clawTimer.reset();
               }

               if (clawTAnimating) {
                   switch (clawTState) {
                       case 0:
                           clawTclosed = !clawTclosed;
                           clawTState++;
                           clawTimer.reset();
                           break;
                       case 1:
                           if (clawTimer.milliseconds() > 200) {
                               clawTServo.setPosition(0.7);
                               clawTState++;
                               clawTimer.reset();
                           }
                           break;
                       case 2:
                           if (clawTimer.milliseconds() > 200) {
                               clawBServo.setPosition(0.35);
                               clawTState++;
                               clawTimer.reset();
                           }
                           break;
                       case 3:
                           if (clawTimer.milliseconds() > 200) {
                               liftTargetPos = -2800;
                               clawTState++;
                               clawTimer.reset();
                           }
                           break;
                       case 4:
                           if (clawTimer.milliseconds() > 1000) {
                               armTargetPos = armTHighBasketPos;
                               clawTState++;
                               clawTimer.reset();
                           }
                           break;
                       case 5:
                           if(clawTimer.milliseconds() > 200){
                               wristTServo.setPosition(0.8);
                               clawTState++;
                               clawTimer.reset();
                           }
                           break;
                       case 6:
                           clawTServo.setPosition(0.7);
                           clawTAnimating = false;
                           break;
                   }
               }

               if (gamepad2.y && !armBbuttonPressed) {
                   transferStarted = false;
                   // Cycle through positions when button is pressed
                   if (armBPosition == 0.4) {
                       armBPosition = 1; // Move to transfer position+
                       if (!transferStarted) {
                           transferPiece = true;
                       }
                       armTargetPos = armTTransferPos;
                       wristBServo.setPosition(0.85);
                       wristTServo.setPosition(0.0);
                       clawBServo.setPosition(0.7);
                       clawTServo.setPosition(0.35);
                       extenderTargetPos = 0;

                   } else {
                       transferPiece = false;
                       armBPosition = 0.4; // Move back to starting position
                   }
                   armBbuttonPressed = true;
               }
               else if (!gamepad2.y) {
                   // Reset button press state when button is released
                   armBbuttonPressed = false;
               }
               armBServo.setPosition(armBPosition);
           }

           //=========================dashboard telemetry==========================

           TelemetryPacket packet = new TelemetryPacket();
           packet.put("Lift1 Position", lift1.getCurrentPosition());
           packet.put("Lift2 Position", lift2.getCurrentPosition());
           packet.put("Lift Target Position", liftTargetPos);
           dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Mode STATUS", modeEnabled);


            telemetry.update();
       }
   }
    private double getHeadingError(double current, double target) {
        double error = target - current;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }
}


