package frc.robot.common;

import edu.wpi.first.wpilibj.Spark;

public class LEDDriver extends Spark {

    public LEDDriver(int port) {
        super(port);
    }

    public static final double AUTONOMOUS = -0.99;
    public static final double TELEOP = -0.61;
}
