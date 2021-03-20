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

public class ShootIntakeShootCommand extends SequentialCommandGroup {
    // shoot 3 balls and move backwards to pick up 3 more and shoot new 3 balls
    private final IntakeSubsystem intakeSubsystem;

    public ShootIntakeShootCommand(
            DriveSubsystem driveSubsystem,
            Odometry odometry,
            Limelight limelight,
            AHRS navx,
            IntakeSubsystem intakeSubsystem,
            ShooterSubsystem shooterSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        TrajectoryConfig config =
                new TrajectoryConfig(
                                Constants.MAX_GENERATION_VELOCITY, Constants.MAX_GENERATION_ACCELERATION)
                        .setKinematics(DriveSubsystem.KINEMATICS)
                        // .addConstraint(new RectangularRegionConstraint(new Translation2d(3.27, -3.5), new
                        // Translation2d(4.343, -2.312), new MaxVelocityConstraint(0.5)))
                        // .addConstraint(new RectangularRegionConstraint(new Translation2d(7.099, -3.48), new
                        // Translation2d(8.229, -2.324), new MaxVelocityConstraint(0.5)))
                        // .addConstraint(new RectangularRegionConstraint(new Translation2d(5.677, -1.969), new
                        // Translation2d(6.883, -0.991), new MaxVelocityConstraint(0.5)))
                        .addConstraint(
                                new DifferentialDriveVoltageConstraint(
                                        DriveSubsystem.FEED_FORWARD,
                                        DriveSubsystem.KINEMATICS,
                                        Constants.MAX_GENERATION_VOLTAGE));
                        // .addConstraint(new CentripetalAccelerationConstraint(10));

        TrajectoryConfig config2 =
                new TrajectoryConfig(0.7, Constants.MAX_GENERATION_ACCELERATION)
                        .addConstraint(
                                new DifferentialDriveVoltageConstraint(
                                        DriveSubsystem.FEED_FORWARD,
                                        DriveSubsystem.KINEMATICS,
                                        Constants.MAX_GENERATION_VOLTAGE))
                        .addConstraint(new CentripetalAccelerationConstraint(5));

        System.out.println("Move to trench front reversed: " + config2.isReversed());
        // Trajectory moveToTrenchFront =
        //         TrajectoryGenerator.generateTrajectory(
        //                 new Pose2d(7.8, 0, Rotation2d.fromDegrees(180)),
        //                 Collections.<Translation2d>emptyList(),
        //                 new Pose2d(5.4, 0, Rotation2d.fromDegrees(180)),
        //                 config2);

        System.out.println("Intake three balls reversed: " + config.isReversed());
        Trajectory intakeThreeBalls = generateTrajectory(config);

        addCommands(
                new InstantCommand(() -> navx.reset()),
                // // new InstantCommand(() -> System.out.println("Start Aligning: " + navx.getAngle())),
                // new ParallelRaceGroup(
                //         new AlignWithLimelightCommand(limelight, driveSubsystem)
                // ),
                // new ParallelRaceGroup(
                //         new RunCommand(() -> shooterSubsystem.shootVoltage(0.5), shooterSubsystem),
                //         new WaitCommand(1)
                // ),
                // // new InstantCommand(() -> System.out.println("Limelight post angle: " +
                // // limelight.getYaw())),
                // // new PrintCommand("Start Automatic Shoot"),
                // new ParallelRaceGroup(
                //         new AutomaticShootCommand(0.5, 3, shooterSubsystem).withTimeout(5),
                //         new RunCommand(() -> this.intakeSubsystem.spin(-7, 0), this.intakeSubsystem)),
                // // new PrintCommand("Gyro Before Align: " + navx.getAngle()),
                // new InstantCommand(() -> this.intakeSubsystem.spin(0, 0), this.intakeSubsystem),
                // new AlignWithGyroCommand(navx, driveSubsystem, 0),
                // // new PrintCommand("Extend intake piston"),
                // new InstantCommand(this.intakeSubsystem::extend, this.intakeSubsystem),
                // // new PrintCommand("Follow intakeThreeBalls and spin intakeSubsystem"),
                new FollowTrajectoryCommand(intakeThreeBalls, odometry, driveSubsystem));
        // // new PrintCommand("Start moveToTrenchFront"),
        // new ChargeAutoCommand(driveSubsystem, 0.6, 0.92),
        // // new PrintCommand("Align w/ limelight"),
        // new AlignWithLimelightCommand(limelight, driveSubsystem),
        // new ParallelRaceGroup(
        //         new WaitCommand(1),
        //         new RunCommand(() -> shooterSubsystem.shootVoltage(0.625), shooterSubsystem)
        // ),
        // // new PrintCommand("Start AutomaticShoot from trench line"),
        // new ParallelRaceGroup(
        //         new AutomaticShootCommand(0.625, 3, shooterSubsystem).withTimeout(5),
        //         new RunCommand(() -> this.intakeSubsystem.spin(-7.5, -5), this.intakeSubsystem)));
        // // new PrintCommand("Finished ShootIntakeShootCommand"));
    }

    private Trajectory generateTrajectory(TrajectoryConfig config) {
        var trajectory =
                TrajectoryGenerator.generateTrajectory(
                        new Pose2d(1.309, -2.388, new Rotation2d(0)),
                        List.of(
                                new Translation2d(2.858, -2.121),
                                new Translation2d(3.797, -2.312),
                                new Translation2d(4.343, -3.036),
                                new Translation2d(4.102, -3.531),
                                new Translation2d(3.556, -3.48),
                                new Translation2d(3.277, -3.137),
                                new Translation2d(3.62, -2.616),
                                new Translation2d(4.889, -2.337),
                                new Translation2d(6.02, -2.134),
                                new Translation2d(6.883, -1.397),
                                new Translation2d(6.248, -0.991),
                                new Translation2d(5.626, -1.321),
                                new Translation2d(5.677, -1.969),
                                new Translation2d(6.299, -2.947),
                                new Translation2d(7.099, -3.48),
                                new Translation2d(4.42, -1.982)),
                        new Pose2d(1.499, -2.159, new Rotation2d(3.14159265358979)),
                        config);
        // var trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
        // List.of(
        //         new Translation2d(2, 2),
        //         new Translation2d(4, 0),
        //         new Translation2d(2, -2)
        // ), new Pose2d(0, 0, new Rotation2d(3.14159265)), config);
        System.out.println(trajectory);
        return trajectory;
    }

    @Override
    public void end(boolean interrupted) {
        this.intakeSubsystem.spin(0, 0);
    }
}
