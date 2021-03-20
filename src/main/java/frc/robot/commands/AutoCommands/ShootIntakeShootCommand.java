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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AlignWithGyroCommand;
import frc.robot.commands.AlignWithLimelightCommand;
import frc.robot.commands.AutomaticShootCommand;
import frc.robot.commands.ChargeAutoCommand;
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
                new TrajectoryConfig(Constants.MAX_GENERATION_VELOCITY, Constants.MAX_GENERATION_ACCELERATION)
                        .addConstraint(
                                new DifferentialDriveVoltageConstraint(
                                        DriveSubsystem.FEED_FORWARD,
                                        DriveSubsystem.KINEMATICS,
                                        Constants.MAX_GENERATION_VOLTAGE))
                                        .addConstraint(new CentripetalAccelerationConstraint(1));

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
        var trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(1.21966560199972, -3.91169170948479, new Rotation2d(0)),
                List.of(
                        new Translation2d(2.90853103735592, -2.83234161921955),
                        new Translation2d(3.72121816414386, -2.08314567421191),
                        new Translation2d(5.13072239966671, -1.90537036522705),
                        new Translation2d(6.41324427162893, -2.73075572837105),
                        new Translation2d(6.99736314400777, -3.67042521871962),
                        new Translation2d(7.91163616164421, -3.91169170948479),
                        new Translation2d(8.41956561588668, -3.73391640049993),
                        new Translation2d(8.73702152478822, -3.17519400083321),
                        new Translation2d(8.54654797944729, -2.52758394667407),
                        new Translation2d(7.82274850715178, -2.07044743785585),
                        new Translation2d(6.95926843493959, -2.46409276489376),
                        new Translation2d(6.56562310790168, -3.26408165532564),
                        new Translation2d(5.91801305374253, -3.8735970004166),
                        new Translation2d(3.20993445878848, -3.80016285998013)),
                        new Pose2d(1.21966560199972, -2.16568421052631, new Rotation2d(3.14159265358979))
                , config);
        // var trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(
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
