package utils;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

public class ArmB{
    private final ServoImplEx armBServo;

    public ArmB(HardwareMap hardwareMap) {
        armBServo = hardwareMap.get(ServoImplEx.class, "armB");

        PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);
        armBServo.setPwmRange(pwmRange);
    }

    public void setPosition(double armBPosition) {
        armBServo.setPosition(armBPosition);
    }

    public double getArmBPosition() {
        return armBServo.getPosition();
    }


    public void increaseArmBPosition(double increment) {
        double newPosition = Math.min(1.0, armBServo.getPosition() + increment); // Ensure the position stays within 0 to 1
        armBServo.setPosition(newPosition);
    }

    public void decreaseArmBPosition(double decrement) {
        double newPosition = Math.max(0.0, armBServo.getPosition() - decrement); // Ensure the position stays within 0 to 1
        armBServo.setPosition(newPosition);
    }


}
