package utils;
public class ServoPose {
    private final double clawT;
    private final double clawB;
    private final double wristB;
    private final double wristT;
    private final double armB;
    private final int armT;
    private int extender;
    private int lifts;
    private final long duration;

    public ServoPose(double clawT, double clawB, double wristB, double wristT, double armB, int armT, long duration) {
        this.clawT = clawT;
        this.clawB = clawB;
        this.wristB = wristB;
        this.wristT = wristT;
        this.armB = armB;
        this.armT = armT;
        this.duration = duration;// Duration in milliseconds
    }

    public ServoPose(double clawT, double clawB, double wristB, double wristT, double armB, int armT, int extender, int lifts, long duration) {
        this.clawT = clawT;
        this.clawB = clawB;
        this.wristB = wristB;
        this.wristT = wristT;
        this.armB = armB;
        this.armT = armT;
        this.extender = extender;
        this.lifts = lifts;
        this.duration = duration;// Duration in milliseconds
    }

    public double getArmBPosition() { return armB; }
    public double getWristBPosition() { return wristB; }
    public double getWristTPosition() { return wristT; }
    public double getClawBPosition() { return clawB;  }
    public double getClawTPosition() { return clawT; }
    public int getArmTPosition(){return armT;}
    public long getDuration() { return duration; }
}

