package frc.robot.commands.AutoCommands;

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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.*;

public class Bounce extends SequentialCommandGroup {
        // shoot 3 balls and move backwards to pick up 3 more and shoot new 3 balls
        private final IntakeSubsystem intakeSubsystem;
                        // .addConstraint(new RectangularRegionConstraint(new Translation2d(3.27, -3.5), new
                        // Translation2d(4.343, -2.312), new MaxVelocityConstraint(0.5)))
                        // .addConstraint(new RectangularRegionConstraint(new Translation2d(7.099, -3.48), new
                        // Translation2d(8.229, -2.324), new MaxVelocityConstraint(0.5)))
                        // .addConstraint(new RectangularRegionConstraint(new Translation2d(5.677, -1.969), new
                        // Translation2d(6.883, -0.991), new MaxVelocityConstraint(0.5)))
                        // .addConstraint(new CentripetalAccelerationConstraint(10));
        public Bounce(
            DriveSubsystem driveSubsystem,
            Odometry odometry,
            Limelight limelight,
            AHRS navx,
            IntakeSubsystem intakeSubsystem,
            ShooterSubsystem shooterSubsystem) {
                this.intakeSubsystem = intakeSubsystem;
                TrajectoryConfig forwardConfig = new TrajectoryConfig(Constants.MAX_GENERATION_VELOCITY, Constants.MAX_GENERATION_ACCELERATION)
                        .setKinematics(DriveSubsystem.KINEMATICS)
                        .addConstraint(
                                new DifferentialDriveVoltageConstraint(
                                        DriveSubsystem.FEED_FORWARD,
                                        DriveSubsystem.KINEMATICS,
                                        Constants.MAX_GENERATION_VOLTAGE));
                TrajectoryConfig reverseConfig = new TrajectoryConfig(Constants.MAX_GENERATION_VELOCITY, Constants.MAX_GENERATION_ACCELERATION)
                        .setKinematics(DriveSubsystem.KINEMATICS)
                        .addConstraint(
                                new DifferentialDriveVoltageConstraint(
                                        DriveSubsystem.FEED_FORWARD,
                                        DriveSubsystem.KINEMATICS,
                                        Constants.MAX_GENERATION_VOLTAGE))
                        .setReversed(true);

                var bounceStart = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(1.524, -2.248, new Rotation2d(0)),
                        List.of(
                                new Translation2d(2.07, -1.842)                     
                        ),
                        new Pose2d(2.274, -0.75, new Rotation2d(1.433484059)),
                        forwardConfig
                );

                var bounceFirstReverse = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(2.274, -0.75, new Rotation2d(1.433484059)),
                        List.of(
                                new Translation2d(2.286, -0.826),
                                new Translation2d(2.705, -2.655),
                                new Translation2d(3.505, -3.759),
                                new Translation2d(4.343, -3.594)
                        ),
                        new Pose2d(4.559, -0.813, new Rotation2d(-1.570796327)),
                        reverseConfig
                );

                var bounceSecondForward = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(4.559, -0.813, new Rotation2d(-1.570796327)),
                        List.of(
                                new Translation2d(4.597, -0.75),
                                new Translation2d(5.055, -3.556),
                                new Translation2d(6.312, -3.569)
                        ),
                        new Pose2d(6.87, -0.724, new Rotation2d(1.273527974)),
                        forwardConfig
                );

                var bounceLastReverse = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(6.87, -0.724, new Rotation2d(1.273527974)),
                        List.of(
                                new Translation2d(6.845, -0.712),
                                new Translation2d(7.289, -1.867)
                        ),
                        new Pose2d(8.305, -2.375, new Rotation2d(3.078026816)),
                        reverseConfig);

                addCommands(
                        new InstantCommand(() -> navx.reset()),
                        new FollowTrajectoryCommand(bounceStart, odometry, driveSubsystem),
                        new FollowTrajectoryCommand(bounceFirstReverse, odometry, driveSubsystem),
                        new FollowTrajectoryCommand(bounceSecondForward, odometry, driveSubsystem),
                        new FollowTrajectoryCommand(bounceLastReverse, odometry, driveSubsystem)
                );
        }

        @Override
        public void end(boolean interrupted) {
                this.intakeSubsystem.spin(0, 0);
        }
}
