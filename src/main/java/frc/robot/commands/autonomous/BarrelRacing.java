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

public class BarrelRacing extends SequentialCommandGroup {
        // shoot 3 balls and move backwards to pick up 3 more and shoot new 3 balls
        private final IntakeSubsystem intakeSubsystem;
                        // .addConstraint(new RectangularRegionConstraint(new Translation2d(3.27, -3.5), new
                        // Translation2d(4.343, -2.312), new MaxVelocityConstraint(0.5)))
                        // .addConstraint(new RectangularRegionConstraint(new Translation2d(7.099, -3.48), new
                        // Translation2d(8.229, -2.324), new MaxVelocityConstraint(0.5)))
                        // .addConstraint(new RectangularRegionConstraint(new Translation2d(5.677, -1.969), new
                        // Translation2d(6.883, -0.991), new MaxVelocityConstraint(0.5)))
                        // .addConstraint(new CentripetalAccelerationConstraint(10));
        public BarrelRacing(
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
                                        Constants.MAX_GENERATION_VOLTAGE))
                        .addConstraint(
                                new CentripetalAccelerationConstraint(1)
                        );
                var barrelRacing = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(1.5244232745452, -2.14663685599222, new Rotation2d(0)),
                        List.of(
                                new Translation2d(2.85773809193167, -2.1212403832801),
                                new Translation2d(3.79740758228023, -2.31171392862102),
                                new Translation2d(4.34343174559089, -3.03551340091653),
                                new Translation2d(4.10216525482571, -3.53074461880294),
                                new Translation2d(3.55614109151506, -3.47995167337869),
                                new Translation2d(3.27677989168171, -3.13709929176503),
                                new Translation2d(3.61963227329537, -2.6164716011665),
                                new Translation2d(4.88945590890154, -2.33711040133314),
                                new Translation2d(6.01959894459102, -2.13393861963616),
                                new Translation2d(6.50213192612137, -1.98155978336342),
                                new Translation2d(6.76879488959866, -1.4228373836967),
                                new Translation2d(6.24816719900013, -0.991097347590612),
                                new Translation2d(5.62595361755311, -1.32125149284821),
                                new Translation2d(5.67674656297736, -1.96886154700736),
                                new Translation2d(6.29896014442438, -2.9466257464241),
                                new Translation2d(7.09894903485626, -3.47995167337869),
                                new Translation2d(8.05131676156089, -3.39106401888626),
                                new Translation2d(8.22909207054575, -2.74345396472712),
                                new Translation2d(7.64497319816692, -2.32441216497708),
                                new Translation2d(6.07039189001527, -2.31171392862102),
                                new Translation2d(4.48311234550756, -2.05774920149979)
                        ),
                        new Pose2d(1.49902680183307, -2.15933509234828, new Rotation2d(3.141592654)),
                        forwardConfig
                );

                addCommands(
                        new InstantCommand(() -> navx.reset()),
                        new FollowTrajectoryCommand(barrelRacing, odometry, driveSubsystem)
                );
        }

        @Override
        public void end(boolean interrupted) {
                this.intakeSubsystem.spin(0, 0);
        }
}
