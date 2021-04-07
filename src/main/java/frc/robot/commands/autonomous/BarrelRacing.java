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

public class BarrelRacing extends SequentialCommandGroup {

    public static final double MAX_GENERATION_VELOCITY = 3.25; // Meters per second
    public static final double MAX_GENERATION_ACCELERATION = 2.2; // Meters per second squared

    public BarrelRacing(
            DriveSubsystem driveSubsystem, Odometry odometry, Limelight limelight, AHRS navx) {
        TrajectoryConfig forwardConfig =
                new TrajectoryConfig(MAX_GENERATION_VELOCITY, MAX_GENERATION_ACCELERATION)
                        .setKinematics(DriveSubsystem.KINEMATICS)
                        .addConstraint(
                                new DifferentialDriveVoltageConstraint(
                                        DriveSubsystem.FEED_FORWARD,
                                        DriveSubsystem.KINEMATICS,
                                        Constants.MAX_GENERATION_VOLTAGE))
                        .addConstraint(new CentripetalAccelerationConstraint(1))
                        .addConstraint(
                                new RectangularRegionConstraint(
                                        new Translation2d(5.348, -2.1),
                                        new Translation2d(6.881, -0.8),
                                        new CentripetalAccelerationConstraint(1)));
        Trajectory barrelRacing =
                TrajectoryGenerator.generateTrajectory(
                        new Pose2d(1.5244232745452, -2.14663685599222, new Rotation2d(0)),
                        List.of(
                                new Translation2d(2.85773809193167, -2.1212403832801),
                                new Translation2d(3.79740758228023, -2.31171392862102),
                                new Translation2d(4.36882821830301, -3.03551340091653),
                                new Translation2d(3.79740758228023, -3.58153756422719),
                                new Translation2d(3.30217636439383, -3.02281516456047),
                                new Translation2d(3.82280405499236, -2.50218747396194),
                                new Translation2d(4.88945590890154, -2.411040133314),
                                new Translation2d(6.12118483543952, -2.324567421191),
                                new Translation2d(6.78149312595472, -1.60061269268157),
                                new Translation2d(6.18467601721983, -0.864114984029995),
                                new Translation2d(5.44817830856825, -1.47363032912095),
                                new Translation2d(6.29896014442438, -2.9466257464241),
                                new Translation2d(7.09894903485626, -3.47995167337869),
                                new Translation2d(7.69576614359116, -3.50534814609082),
                                new Translation2d(8.27988501597, -2.89583280099986),
                                new Translation2d(7.64497319816692, -2.32441216497708),
                                new Translation2d(6.07039189001527, -2.31171392862102),
                                new Translation2d(4.48311234550756, -2.05774920149979)),
                        new Pose2d(1.49902680183307, -1.85933509234828, new Rotation2d(3.141592654)),
                        forwardConfig);

        addCommands(
                new InstantCommand(() -> navx.reset()),
                new FollowTrajectoryCommand(barrelRacing, odometry, driveSubsystem));
    }
}
