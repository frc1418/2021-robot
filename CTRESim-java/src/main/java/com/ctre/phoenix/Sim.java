package com.ctre.phoenix;

import edu.wpi.first.wpilibj.RobotBase;

public class Sim {
    static {
        if (RobotBase.isReal()) {
            throw new RuntimeException("CTRE simulation library cannot be used on a real robot.");
        }
    }
}
