package utils;

import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ExtenderPD {
    private Motor extenderMotor;
    private PDController extenderPD;
    private double extenderTargetPos;


    public ExtenderPD(HardwareMap hardwareMap){
        extenderMotor = new Motor(hardwareMap, "extendB");
        extenderPD = new PDController(0, 0);
        extenderTargetPos = 0;

    }

    public void resetEncoder() { extenderMotor.resetEncoder(); }
    public void setTargetPosition(double position) { extenderTargetPos = position; }

    public boolean isAtTarget(){
        double error = Math.abs(extenderTargetPos - extenderMotor.getCurrentPosition());
        return error < 20;
    }

    public void setPID(double kP, double kI, double kD){
        extenderPD.setPID(kP, kI, kD);
    }

    public void update(){
        double extenderPos = extenderMotor.getCurrentPosition();
        double extenderPower = extenderPD.calculate(extenderPos, extenderTargetPos);
        extenderMotor.set(extenderPower);
    }

    public double getCurrentPosition() { return extenderMotor.getCurrentPosition(); }

    public double getTargetPosition() { return  extenderTargetPos; }
}
