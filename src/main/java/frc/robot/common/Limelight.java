package frc.robot.common;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    private static final NetworkTable table = NetworkTableInstance.getDefault()
        .getTable("limelight");

    public double getYaw() {
        return table.getEntry("tx")
            .getDouble(0);
    }

    public void setLedMode(int ledMode) {
        table.getEntry("ledMode")
            .setNumber(ledMode);
    }

    public static class Constants {

        // LED MODES /limelight/ledMode
        public static final int LED_MODE_FROM_PIPELINE = 0;
        public static final int LED_MODE_FORCE_OFF = 1;
        public static final int LED_MODE_FORCE_BLINK = 2;
        public static final int LED_MODE_FORCE_ON = 3;
    }
}
