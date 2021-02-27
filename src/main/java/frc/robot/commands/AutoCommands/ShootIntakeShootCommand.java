package frc.robot.commands.AutoCommands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AlignWithGyroCommand;
import frc.robot.commands.AlignWithLimelightCommand;
import frc.robot.commands.AutomaticShootCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.common.Limelight;
import frc.robot.common.Odometry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.HashMap;

public class ShootIntakeShootCommand extends SequentialCommandGroup {
    // shoot 3 balls and move backwards to pick up 3 more and shoot new 3 balls
    private String TRAJECTORY_1_NAME = "intakeThreeBalls";
    private String TRAJECTORY_2_NAME = "threeBallsTrenchToFront";


    public ShootIntakeShootCommand(
            DriveSubsystem driveSubsystem,
            Odometry odometry,
            Limelight limelight,
            AHRS navx,
            IntakeSubsystem intakeSubsystem,
            ShooterSubsystem shooterSubsystem,
            HashMap<String, Trajectory> trajectories) {
        Trajectory intakeThreeBalls = trajectories.get(TRAJECTORY_1_NAME);
        Trajectory moveToTrenchFront = trajectories.get(TRAJECTORY_2_NAME);

        addCommands(
                new PrintCommand("Start Aligning"),
                new AlignWithLimelightCommand(limelight, driveSubsystem),
                new PrintCommand("Start Automatic Shoot"),
                new ParallelDeadlineGroup(
                        new AutomaticShootCommand(
                                ShooterSubsystem.Constants.INITITATION_LINE_VEL, 3, shooterSubsystem),
                        new InstantCommand(() -> intakeSubsystem.spin(-0.5, 0), intakeSubsystem)),
                new PrintCommand("Gyro Before Align: " + navx.getAngle()),
                new AlignWithGyroCommand(navx, driveSubsystem, 0),
                new PrintCommand("Gyro After Align: " + navx.getAngle()),
                new PrintCommand("Extend intake piston"),
                new InstantCommand(intakeSubsystem::extend, intakeSubsystem),
                new PrintCommand("Follow intakeThreeBalls and spin intakeSubsystem"),
                new ParallelDeadlineGroup(
                        new FollowTrajectoryCommand(intakeThreeBalls, odometry, driveSubsystem),
                        new InstantCommand(() -> intakeSubsystem.spin(-0.5, -0.7, true), intakeSubsystem)),
                new PrintCommand("Start moveToTrenchFront"),
                new FollowTrajectoryCommand(moveToTrenchFront, odometry, driveSubsystem),
                new PrintCommand("Start AutomaticShoot from trench line"),
                new ParallelDeadlineGroup(
                        new AutomaticShootCommand(ShooterSubsystem.Constants.TRENCH_LINE_VEL, 3, shooterSubsystem),
                        new InstantCommand(() -> intakeSubsystem.spin(-0.5, -0.7, true), intakeSubsystem)),
                new PrintCommand("Finished ShootIntakeShootCommand"));
    }
}
