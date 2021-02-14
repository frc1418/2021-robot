/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Drivetrain devices
    public static final int FRONT_LEFT_MOTOR = 1;
    public static final int FRONT_RIGHT_MOTOR = 4;
    public static final int REAR_LEFT_MOTOR = 3;
    public static final int REAR_RIGHT_MOTOR = 6;

    // Shooter devices
    public static final int SHOOTER_MOTOR_1 = 10;
    public static final int SHOOTER_MOTOR_2 = 12;
    public static final int SHOOTER_SOLENOID_PORT = 0;
    public static final int SHOOTER_ULTRASONIC_TRIG = 0;
    public static final int SHOOTER_ULTRASONIC_ECHO = 1;

    // Intake devices
    public static final int UPPER_INTAKE_MOTOR = 1;
    public static final int BOTTOM_INTAKE_MOTOR = 40;
    public static final int INTAKE_SWITCH = 2;
    public static final int INTAKE_SOLENOID_FWD = 4;
    public static final int INTAKE_SOLENOID_REV = 5;

    // Control panel devices
    public static final int CONTROL_PANEL_MOTOR = 15;
    public static final int CONTROL_PANEL_SOLENOID_FWD = 3;
    public static final int CONTROL_PANEL_SOLENOID_REV = 1;

    // RobotContainer object configuration
    public static final double DRIVE_GEARING = 7.56;  // Meters
    public static final double DRIVE_WHEEL_DIAMETER = 0.1524;  // Meters
    public static final double DRIVE_ENCODER_CONSTANT = (1 / DRIVE_GEARING) * DRIVE_WHEEL_DIAMETER * Math.PI;

    // DriveSubsystem constants
    public static final double TRACK_WIDTH = 0.481;  // Meters
    public static final double WHEEL_BASE = 0.5969;  // Meters
    public static final double DRIVE_KS = 0.238;  // Volts
    public static final double DRIVE_KV = 1.99;  // Volt seconds per meter
    public static final double DRIVE_KA = 0.504;  // Volt seconds squared per meter

    // Trajectory generation constraints
    public static final double MAX_GENERATION_VOLTAGE = 8;  // Volts
    public static final double MAX_GENERATION_VELOCITY = 2.3;  // Meters per second
    public static final double MAX_GENERATION_ACCELERATION = 1.5;  // Meters per second squared
}
