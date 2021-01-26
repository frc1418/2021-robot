package frc.robot.common;

import edu.wpi.first.wpilibj.Spark;

public class LEDDriver extends Spark {

    public LEDDriver(int port) {
        super(port);
    }

    public static final double autonomous = -0.99;
    public static final double teleop = -0.61;
}
