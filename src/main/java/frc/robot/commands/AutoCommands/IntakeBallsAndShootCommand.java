package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignWithLimelightCommand;
import frc.robot.commands.AutomaticShootCommand;
import frc.robot.common.Limelight;
import frc.robot.common.Odometry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.HashMap;

public class IntakeBallsAndShootCommand extends SequentialCommandGroup {
    // move to trench, pick up two balls, move to front of trench, fire 5 balls

    public IntakeBallsAndShootCommand(
            DriveSubsystem driveSubsystem,
            Odometry odometry,
            Limelight limelight,
            ShooterSubsystem shooterSubsystem,
            IntakeSubsystem intakeSubsystem,
            HashMap<String, Trajectory> trajectories) {
        addCommands(
                new IntakeTwoBallsCommand(
                        driveSubsystem, odometry, limelight, shooterSubsystem, intakeSubsystem, trajectories),
                new AlignWithLimelightCommand(limelight, driveSubsystem),
                new AutomaticShootCommand(ShooterSubsystem.Constants.TRENCH_LINE_VEL, 5, shooterSubsystem));
    }
}
