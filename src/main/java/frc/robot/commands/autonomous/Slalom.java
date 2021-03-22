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
import edu.wpi.first.wpilibj.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.RectangularRegionConstraint;
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

public class Slalom extends SequentialCommandGroup {
        // shoot 3 balls and move backwards to pick up 3 more and shoot new 3 balls
        private final IntakeSubsystem intakeSubsystem;
                        // .addConstraint(new RectangularRegionConstraint(new Translation2d(3.27, -3.5), new
                        // Translation2d(4.343, -2.312), new MaxVelocityConstraint(0.5)))
                        // .addConstraint(new RectangularRegionConstraint(new Translation2d(7.099, -3.48), new
                        // Translation2d(8.229, -2.324), new MaxVelocityConstraint(0.5)))
                        // .addConstraint(new RectangularRegionConstraint(new Translation2d(5.677, -1.969), new
                        // Translation2d(6.883, -0.991), new MaxVelocityConstraint(0.5)))
                        // .addConstraint(new CentripetalAccelerationConstraint(10));
        public Slalom(
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
                                new CentripetalAccelerationConstraint(2)
                        )
                        .addConstraint(
                                new RectangularRegionConstraint(
                                        new Translation2d(6.743, -3.874),
                                        new Translation2d(8.699, -2.07),
                                        new CentripetalAccelerationConstraint(1)        
                                )        
                        );

                var slalom = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(1.53712151090126, -3.8735970004166, new Rotation2d(0)),
                        List.of(
                                new Translation2d(2.43869629218164, -2.76885043743924),
                                new Translation2d(3.50534814609082, -2.41329981946951),
                                new Translation2d(4.99104179975003, -2.32441216497708),
                                new Translation2d(6.47673545340924, -2.57837689209831),
                                new Translation2d(6.99736314400777, -3.47995167337869),
                                new Translation2d(7.6830679072351, -3.86089876406054),
                                new Translation2d(8.43226385224274, -2.95932398278016),
                                new Translation2d(7.7846537980836, -2.05774920149979),
                                new Translation2d(6.98466490765171, -2.51488571031801),
                                new Translation2d(6.74339841688654, -3.39106401888626),
                                new Translation2d(5.91801305374253, -3.8735970004166),
                                new Translation2d(3.8101058186363, -3.86089876406054),
                                new Translation2d(2.33711040133314, -3.09900458269684)
                        ),
                        new Pose2d(1.34664796556033, -2.1847315650604, new Rotation2d(3.141592654)),
                        forwardConfig
                );

                addCommands(
                        new InstantCommand(() -> navx.reset()),
                        new FollowTrajectoryCommand(slalom, odometry, driveSubsystem)
                );
        }

        @Override
        public void end(boolean interrupted) {
                this.intakeSubsystem.spin(0, 0);
        }
}
