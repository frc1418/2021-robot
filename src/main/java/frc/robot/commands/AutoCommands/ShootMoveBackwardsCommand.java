package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignWithLimelightCommand;
import frc.robot.commands.AutomaticShootCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.common.Odometry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.HashMap;

public class ShootMoveBackwardsCommand extends SequentialCommandGroup {
    // shoot 3 balls and move backwards off initiation line
    private String TRAJECTORY_NAME = "MoveOffInitiationLine";

    public ShootMoveBackwardsCommand(
            DriveSubsystem driveSubsystem,
            Odometry odometry,
            Limelight limelight,
            ShooterSubsystem shooterSubsystem,
            HashMap<String, Trajectory> trajectories) {
        Trajectory moveOffInitiationLine = trajectories.get(TRAJECTORY_NAME);
        addCommands(
                new AlignWithLimelightCommand(limelight, driveSubsystem),
                new AutomaticShootCommand(ShooterSubsystem.Constants.INITITATION_LINE_VEL, 3, shooterSubsystem),
                new FollowTrajectoryCommand(moveOffInitiationLine, odometry, driveSubsystem));
    }
}
