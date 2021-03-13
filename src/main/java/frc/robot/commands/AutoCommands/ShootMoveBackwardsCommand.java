package frc.robot.commands.AutoCommands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignWithGyroCommand;
import frc.robot.commands.AlignWithLimelightCommand;
import frc.robot.commands.AutomaticShootCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.HashMap;

public class ShootMoveBackwardsCommand extends SequentialCommandGroup {
    // shoot 3 balls and move backwards off initiation line
    private String TRAJECTORY_NAME = "moveOffInitiationLine";

    public ShootMoveBackwardsCommand(
            DriveSubsystem driveSubsystem,
            Odometry odometry,
            Limelight limelight,
            AHRS navx,
            ShooterSubsystem shooterSubsystem,
            HashMap<String, Trajectory> trajectories) {
        Trajectory moveOffInitiationLine = trajectories.get(TRAJECTORY_NAME);
        addCommands(
                new PrintCommand("Start Aligning"),
                new AlignWithLimelightCommand(limelight, driveSubsystem),
                new PrintCommand("Start Automatic Shoot"),
                new AutomaticShootCommand(
                        ShooterSubsystem.Constants.INITITATION_LINE_VEL, 3, shooterSubsystem),
                new PrintCommand("Gyro Align: " + navx.getAngle()),
                new AlignWithGyroCommand(navx, driveSubsystem, 0),
                new PrintCommand("Start moveOffInitLine"),
                new FollowTrajectoryCommand(moveOffInitiationLine, odometry, driveSubsystem),
                new PrintCommand("Finish following trajectory"));
    }
}
