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
import edu.wpi.first.wpilibj.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import java.util.List;

public class Slalom extends SequentialCommandGroup {
    public static final double MAX_GENERATION_VELOCITY = 3.5; // Meters per second
    public static final double MAX_GENERATION_ACCELERATION = 2.2; // Meters per second squared

    public Slalom(DriveSubsystem driveSubsystem, Odometry odometry, Limelight limelight, AHRS navx) {
        TrajectoryConfig forwardConfig =
                new TrajectoryConfig(MAX_GENERATION_VELOCITY, MAX_GENERATION_ACCELERATION)
                        .setKinematics(DriveSubsystem.KINEMATICS)
                        .addConstraint(
                                new DifferentialDriveVoltageConstraint(
                                        DriveSubsystem.FEED_FORWARD,
                                        DriveSubsystem.KINEMATICS,
                                        Constants.MAX_GENERATION_VOLTAGE))
                        .addConstraint(new CentripetalAccelerationConstraint(4))
                        .addConstraint(
                                new RectangularRegionConstraint(
                                        new Translation2d(6.743, -3.874),
                                        new Translation2d(8.7, -2.07),
                                        new CentripetalAccelerationConstraint(2.5)));

        Trajectory slalom =
                TrajectoryGenerator.generateTrajectory(
                        new Pose2d(1.156, -3.886, new Rotation2d(0)),
                        List.of(
                                new Translation2d(2.43869629218164, -2.76885043743924),
                                new Translation2d(3.50534814609082, -2.41329981946951),
                                new Translation2d(4.99104179975003, -2.32441216497708),
                                new Translation2d(6.47673545340924, -2.57837689209831),
                                new Translation2d(6.99736314400777, -3.47995167337869),
                                new Translation2d(7.874, -3.886),
                                new Translation2d(8.674, -2.95932398278016),
                                new Translation2d(7.874, -2.363),
                                new Translation2d(7.378, -2.629),
                                new Translation2d(7.162, -3.226),
                                new Translation2d(6.299, -4.039),
                                new Translation2d(3.84820052770448, -3.75931287321205),
                                new Translation2d(2.769, -3.277),
                                new Translation2d(2.147, -2.388)),
                        new Pose2d(1.258, -1.918, new Rotation2d(3.14159)),
                        forwardConfig);

        addCommands(
                new InstantCommand(() -> navx.reset()),
                new FollowTrajectoryCommand(slalom, odometry, driveSubsystem));
    }
}
