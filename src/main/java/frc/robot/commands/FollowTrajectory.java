package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectory extends RamseteCommand {

    public FollowTrajectory(Trajectory trajectory, Odometry odometry, DriveSubsystem driveSubsystem) {
        super(
                trajectory,
                odometry::getPose,
                new RamseteController(),
                new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA),
                DriveSubsystem.KINEMATICS,
                odometry::getWheelSpeeds,
                new PIDController(0.00, 0, 0),
                new PIDController(0.00, 0, 0),
                // RamseteCommand passes volts to the callback
                driveSubsystem::tankDriveVolts,
                driveSubsystem);
    }
}
