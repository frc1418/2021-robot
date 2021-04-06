package frc.robot.common;

import edu.wpi.first.wpilibj.Spark;

public class LEDDriver extends Spark {

    public LEDDriver(int port) {
        super(port);
    }

    public void setRestingColor() {}

    public static final double AUTONOMOUS = -0.99; //
    public static final double TELEOP = -0.43; //
    public static final double DISABLED = 0.93; // Solid white
    public static final double SHOOTING = 0.01; // Uses Color 1
}
