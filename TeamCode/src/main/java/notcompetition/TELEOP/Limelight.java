package notcompetition.TELEOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.HashMap;

public class Limelight {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Servo clawBServo;

    public Limelight(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.clawBServo = hardwareMap.get(Servo.class, "clawB"); // Adjust name as needed
    }

    public void updateTelemetry() {
        HashMap<String, Double> limelightData = getLimelightData();

        double tx = limelightData.getOrDefault("tx", 0.0); // Horizontal offset
        double ty = limelightData.getOrDefault("ty", 0.0); // Vertical offset
        double ts = limelightData.getOrDefault("ts", 0.0); // Skew/rotation

        int pipeline = determinePipeline(limelightData);
        setPipeline(pipeline);

        double angle = calculateAngle(tx, ty, ts);
        adjustClaw(angle);

        telemetry.addData("Detected Angle", angle);
        telemetry.addData("Active Pipeline", pipeline);
        telemetry.update();
    }

    private HashMap<String, Double> getLimelightData() {
        HashMap<String, Double> data = new HashMap<>();

        // Simulated data retrieval (replace with actual network table queries if needed)
        data.put("tx", getNetworkTableValue("tx"));
        data.put("ty", getNetworkTableValue("ty"));
        data.put("ts", getNetworkTableValue("ts"));
        data.put("tv", getNetworkTableValue("tv")); // Target valid
        data.put("tID", getNetworkTableValue("tID")); // Target ID or classification (mocked)

        return data;
    }

    private double getNetworkTableValue(String key) {
        // Placeholder for actual Limelight API integration
        return 0.0; // Replace with NetworkTable or HTTP GET request
    }

    private double calculateAngle(double tx, double ty, double ts) {
        // If Limelight provides direct skew (ts), use it
        if (ts != 0.0) {
            return ts;
        }

        // Otherwise, estimate angle using trigonometry
        return Math.toDegrees(Math.atan2(ty, tx));
    }

    private int determinePipeline(HashMap<String, Double> limelightData) {
        // Assume tID is a target identifier for different colors
        double targetID = limelightData.getOrDefault("tID", -1.0);

        if (targetID == 0) { // Red
            return 0;
        } else if (targetID == 1) { // Yellow
            return 1;
        } else if (targetID == 2) { // Blue
            return 2;
        }
        return 0; // Default to red pipeline
    }

    private void setPipeline(int pipeline) {
        // Placeholder for setting the Limelight pipeline
        // Replace with actual API call to set pipeline
        telemetry.addData("Switching to Pipeline", pipeline);
    }

    private void adjustClaw(double angle) {
        // Convert angle into a servo position (assuming servo range is 0-1)
        double servoPosition = Math.max(0, Math.min(1, (angle + 90) / 180));
        clawBServo.setPosition(servoPosition);
        telemetry.addData("Claw Position", servoPosition);
    }
}
