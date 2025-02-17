package utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

public class Claw {
    private final ServoImplEx clawBServo;
    private final ServoImplEx clawTServo;

    // Define specific positions for open and closed states
    private static final double GRAB_POSITION = 0.65; // Adjust this to the desired closed position
    private static final double RELEASE_POSITION = 0.35; // Adjust this to the desired open position

    public Claw(HardwareMap hardwareMap) {
        clawBServo = hardwareMap.get(ServoImplEx.class, "clawB");
        clawTServo = hardwareMap.get(ServoImplEx.class, "clawT");

        PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);
        clawBServo.setPwmRange(pwmRange);
        clawTServo.setPwmRange(pwmRange);
    }

//    public void grab() {
//        clawServo.setPosition(GRAB_POSITION);
//    }
//
//    public void release() {
//        clawServo.setPosition(RELEASE_POSITION);
//    }

    public double getClawBPosition() {
        return clawBServo.getPosition();
    }

    public double getClawTPosition() {
        return clawTServo.getPosition();
    }

    public void setPosition(double clawBPosition, double clawTPosition) {
        clawBServo.setPosition(clawBPosition);
        clawTServo.setPosition(clawTPosition);
    }
}