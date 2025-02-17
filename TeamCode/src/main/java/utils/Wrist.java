package utils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

import com.pedropathing.pathgen.Point;

public class Wrist {
    private final ServoImplEx wristBServo;
    private final ServoImplEx wristTServo;

    // Define the preset positions
    private static final double POSITION_1 = 0; // Adjust these values as needed
    private static final double POSITION_2 = 0.5;
    private static final double POSITION_3 = 1;

    private int positionIndex = 0; // Tracks the current position in the cycle

    public Wrist(HardwareMap hardwareMap) {
        wristBServo = hardwareMap.get(ServoImplEx.class, "wristB");
        wristTServo = hardwareMap.get(ServoImplEx.class, "wristT");

        PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);
        wristTServo.setPwmRange(pwmRange);
        wristTServo.setPwmRange(pwmRange);

        // Initialize to the first position
        setPosition(POSITION_1, POSITION_1);
    }

    // Method to cycle through positions
    public void cyclePosition() {
        positionIndex = (positionIndex + 1) % 3; // Cycle between 0, 1, and 2
        switch (positionIndex) {
            case 0:
                setPosition(POSITION_1, POSITION_1);
                break;
            case 1:
                setPosition(POSITION_2, POSITION_2);
                break;
            case 2:
                setPosition(POSITION_3, POSITION_3);
                break;
        }
    }

    public void setPosition(double wristBPosition, double wristTPosition) {
        wristBServo.setPosition(wristBPosition);
        wristTServo.setPosition(wristTPosition);
    }

    public double getWristBPosition() {
        return wristBServo.getPosition();
    }

    public double getWristTPosition() {
        return wristTServo.getPosition();
    }
}
