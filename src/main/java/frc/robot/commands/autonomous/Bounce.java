package frc.robot.commands.autonomous;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import java.util.List;

public class Bounce extends SequentialCommandGroup {
    public static final double MAX_GENERATION_VELOCITY = 2; // Meters per second
    public static final double MAX_GENERATION_ACCELERATION = 2; // Meters per second squared

    public Bounce(DriveSubsystem driveSubsystem, Odometry odometry, Limelight limelight, AHRS navx) {
        TrajectoryConfig forwardConfig =
                new TrajectoryConfig(MAX_GENERATION_VELOCITY, MAX_GENERATION_ACCELERATION)
                        .setKinematics(DriveSubsystem.KINEMATICS)
                        .addConstraint(
                                new DifferentialDriveVoltageConstraint(
                                        DriveSubsystem.FEED_FORWARD,
                                        DriveSubsystem.KINEMATICS,
                                        Constants.MAX_GENERATION_VOLTAGE))
                        .addConstraint(new CentripetalAccelerationConstraint(1));
        TrajectoryConfig reverseConfig =
                new TrajectoryConfig(MAX_GENERATION_VELOCITY, MAX_GENERATION_ACCELERATION)
                        .setKinematics(DriveSubsystem.KINEMATICS)
                        .addConstraint(
                                new DifferentialDriveVoltageConstraint(
                                        DriveSubsystem.FEED_FORWARD,
                                        DriveSubsystem.KINEMATICS,
                                        Constants.MAX_GENERATION_VOLTAGE))
                        .addConstraint(new CentripetalAccelerationConstraint(1))
                        .setReversed(true);

        Trajectory bounceStart =
                TrajectoryGenerator.generateTrajectory(
                        new Pose2d(1.08, -2.274, new Rotation2d(0)),
                        List.of(new Translation2d(2.197, -1.766)),
                        new Pose2d(2.274, -0.8, new Rotation2d(1.57079632679)),
                        forwardConfig);

        Trajectory bounceFirstReverse =
                TrajectoryGenerator.generateTrajectory(
                        new Pose2d(2.274, -0.8, new Rotation2d(1.57079632679)),
                        List.of(
                                new Translation2d(3.493, -3.188),
                                new Translation2d(4.001, -3.67),
                                new Translation2d(4.623, -3.378)),
                        new Pose2d(4.712, -0.902, new Rotation2d(-1.570796327)),
                        reverseConfig);

        Trajectory bounceSecondForward =
                TrajectoryGenerator.generateTrajectory(
                        new Pose2d(4.712, -0.902, new Rotation2d(-1.570796327)),
                        List.of(
                                new Translation2d(4.826, -3.353),
                                new Translation2d(5.194, -3.912),
                                new Translation2d(6.604, -3.924),
                                new Translation2d(6.959, -2.337)),
                        new Pose2d(7.4, -0.724, new Rotation2d(1.57079632679)),
                        forwardConfig);

        Trajectory bounceLastReverse =
                TrajectoryGenerator.generateTrajectory(
                        new Pose2d(7.4, -0.724, new Rotation2d(1.57079632679)),
                        List.of(new Translation2d(7.366, -2.312)),
                        new Pose2d(8.305, -2.375, new Rotation2d(3.141592653)),
                        reverseConfig);

        addCommands(
                new InstantCommand(() -> navx.reset()),
                new FollowTrajectoryCommand(bounceStart, odometry, driveSubsystem),
                new FollowTrajectoryCommand(bounceFirstReverse, odometry, driveSubsystem, false),
                new FollowTrajectoryCommand(bounceSecondForward, odometry, driveSubsystem, false),
                new FollowTrajectoryCommand(bounceLastReverse, odometry, driveSubsystem, false));
    }
}
