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

    // Intake devices
    public static final int UPPER_INTAKE_MOTOR = 1;
    public static final int BOTTOM_INTAKE_MOTOR = 40;
    public static final int INTAKE_SWITCH = 2;
    public static final int INTAKE_SOLENOID_FWD = 2;
    public static final int INTAKE_SOLENOID_REV = 1;

    // Control panel devices
    public static final int CONTROL_PANEL_MOTOR = 15;
    public static final int CONTROL_PANEL_SOLENOID_FWD = 3;
    public static final int CONTROL_PANEL_SOLENOID_REV = 4;
}
