package utils;

import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmTPD {
    private Motor armMotor;
    private PDController armPD;
    private double armTargetPos;
    public static double armT_Kd = 0.00001, armT_Kp = 0.001;
    public ArmTPD(HardwareMap hardwareMap) {
        armMotor = new Motor(hardwareMap, "armT");
        armPD = new PDController(armT_Kp, armT_Kd);
        armTargetPos = 0;

    }
    public void resetEncoder() { armMotor.resetEncoder(); }
    public void setTargetPosition(double position) {
        armTargetPos = position;
    }

    public void setPID(double kP, double kI, double kD){
        armPD.setPID(kP, kI, kD);
    }

    public void update() {
        double armPos = armMotor.getCurrentPosition();
        double armPower = armPD.calculate(armPos, armTargetPos);
        armMotor.set(armPower);
    }

    public boolean isAtTarget(){
        double error = Math.abs(armTargetPos - armMotor.getCurrentPosition());
        return error < 100;
    }

    public double getCurrentPosition() {
        return armMotor.getCurrentPosition();
    }

    public double getTargetPosition() {
        return armTargetPos;
    }
}
