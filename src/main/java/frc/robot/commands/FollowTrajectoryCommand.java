package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.common.Odometry;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectoryCommand extends SequentialCommandGroup {


    public FollowTrajectoryCommand(Trajectory trajectory, Odometry odometry, DriveSubsystem driveSubsystem) {
        this(trajectory, odometry, driveSubsystem, true);
    }

    public FollowTrajectoryCommand(Trajectory trajectory, Odometry odometry, DriveSubsystem driveSubsystem, boolean resetOdometry) {
        if (resetOdometry) {
            odometry.reset(trajectory.getInitialPose());
        }
        
        addCommands(
            new RamseteCommand(
                trajectory,
                odometry::getPose,
                new RamseteController(),
                new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA),
                DriveSubsystem.KINEMATICS,
                odometry::getWheelSpeeds,
                new PIDController(0.06, 0, 0),
                new PIDController(0.06, 0, 0),
                driveSubsystem::tankDriveVolts,
                driveSubsystem),
            new InstantCommand(() -> driveSubsystem.tankDriveVolts(0, 0))
        );
    }
}
