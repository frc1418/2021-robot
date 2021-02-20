package frc.robot.commands;

import frc.robot.common.TrajectoryLoader;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousCommand extends SequentialCommandGroup {
    // private final TrajectoryLoader trajectoryLoader = new TrajectoryLoader();
    
    // public AutonomousCommand() {
    //     this.addCommands(

    //     );
    // }

        
    //     // An example trajectory to follow.  All units in meters.
    //     Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(
    //             //TODO fix turning
    //             new Translation2d(1, 1),
    //             new Translation2d(2, -1)
    //         ),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(1, 0, new Rotation2d(0)),
    //         // Pass config
    //         config
    //     );

    //     RamseteCommand ramseteCommand = new RamseteCommand(
    //         testTrajectory,
    //         odometry::getPose,
    //         new RamseteController(),
    //         new SimpleMotorFeedforward(DRIVE_KS,
    //                                 DRIVE_KV,
    //                                 DRIVE_KA),
    //         DriveSubsystem.KINEMATICS,
    //         odometry::getWheelSpeeds,
    //         new PIDController(0.00, 0, 0),
    //         new PIDController(0.00, 0, 0),
    //         // RamseteCommand passes volts to the callback
    //         driveSubsystem::tankDriveVolts,
    //         driveSubsystem
    //     );

    //     // Reset odometry to the starting pose of the trajectory.
    //     odometry.reset(testTrajectory.getInitialPose());

    //     // Run path following command, then stop at the end.
    //     return ramseteCommand.andThen(() -> driveTrain.tankDrive(0, 0));
}
