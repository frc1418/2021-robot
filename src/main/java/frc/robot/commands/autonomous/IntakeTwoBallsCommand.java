package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.HashMap;

public class IntakeTwoBallsCommand extends SequentialCommandGroup {
    public IntakeTwoBallsCommand(
            DriveSubsystem driveSubsystem,
            Odometry odometry,
            Limelight limelight,
            ShooterSubsystem shooterSubsystem,
            IntakeSubsystem intakeSubsystem,
            HashMap<String, Trajectory> trajectories) {
        Trajectory startThroughTwoBalls = trajectories.get("StartThroughTwoBalls");
        Trajectory moveToTrenchFront = trajectories.get("TwoBallsToFrontOfTrench");
        addCommands(
                new InstantCommand(intakeSubsystem::extend, intakeSubsystem),
                new ParallelCommandGroup(
                        new FollowTrajectoryCommand(startThroughTwoBalls, odometry, driveSubsystem),
                        new InstantCommand(() -> intakeSubsystem.spin(-0.5, -0.7), intakeSubsystem)),
                new FollowTrajectoryCommand(moveToTrenchFront, odometry, driveSubsystem));
    }
}
