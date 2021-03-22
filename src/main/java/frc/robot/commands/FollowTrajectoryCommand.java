package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.common.Odometry;
import frc.robot.subsystems.DriveSubsystem;

public class FollowTrajectoryCommand extends SequentialCommandGroup {

    private Odometry odometry;
    private Trajectory trajectory;
    private boolean resetOdometry = true;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("/autonomous");
    private NetworkTableEntry leftMeasurement = table.getEntry("leftMeasurment");
    private NetworkTableEntry leftReference = table.getEntry("leftReference");
    private NetworkTableEntry leftVoltage = table.getEntry("leftVoltage");
    private NetworkTableEntry rightMeasurement = table.getEntry("rightMeasurment");
    private NetworkTableEntry rightReference = table.getEntry("rightReference");
    private NetworkTableEntry rightVoltage = table.getEntry("leftVoltage");

    public FollowTrajectoryCommand(
            Trajectory trajectory, Odometry odometry, DriveSubsystem driveSubsystem) {
        this(trajectory, odometry, driveSubsystem, true);

        leftMeasurement.setDefaultDouble(0);
        leftReference.setDefaultDouble(0);
        rightMeasurement.setDefaultDouble(0);
        rightReference.setDefaultDouble(0);
    }

    public FollowTrajectoryCommand(
            Trajectory trajectory,
            Odometry odometry,
            DriveSubsystem driveSubsystem,
            boolean resetOdometry) {

        this.resetOdometry = resetOdometry;
        this.trajectory = trajectory;
        this.odometry = odometry;

        RamseteController ramseteController = new RamseteController();
        // ramseteController.setEnabled(false);
        PIDController leftController = new PIDController(1, 0, 0);
        PIDController rightController = new PIDController(1, 0, 0);
        addCommands(
                new RamseteCommand(
                        trajectory,
                        odometry::getPose,
                        ramseteController,
                        new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA),
                        DriveSubsystem.KINEMATICS,
                        odometry::getWheelSpeeds,
                        leftController,
                        rightController,
                        (leftVolts, rightVolts) -> {
                            driveSubsystem.tankDriveVolts(leftVolts, rightVolts);

                            leftVoltage.setNumber(leftVolts);
                            leftMeasurement.setNumber(odometry.getWheelSpeeds().leftMetersPerSecond);
                            leftReference.setNumber(leftController.getSetpoint());

                            rightVoltage.setNumber(rightVolts);
                            rightMeasurement.setNumber(odometry.getWheelSpeeds().rightMetersPerSecond);
                            rightReference.setNumber(rightController.getSetpoint());
                        },
                        driveSubsystem),
                new InstantCommand(() -> driveSubsystem.tankDriveVolts(0, 0)));
    }

    @Override
    public void initialize() {
        super.initialize();
        if (resetOdometry) {
            odometry.reset(trajectory.getInitialPose());
        }
    }
}
