package frc.robot.commands;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.Limelight;
import frc.robot.subsystems.DriveSubsystem; 

public class AlignCommand extends CommandBase {

    private static final double Ki = 0;
    private static final double Kd = 0;
    private static double Kp = 1.0;
    private boolean useLimelight;
    private double turnToAngle;
    // Aka "Amy"
    private final PIDController aimyAim = new PIDController(Kp, Ki, Kd);
    private final Limelight limelight;
    private final DriveSubsystem driveSubsystem;
    private final Object gyro;

    public AlignCommand(Limelight limelight, DriveSubsystem driveSubsystem) {
        this.limelight = limelight;
        useLimelight = true;
        this.driveSubsystem = driveSubsystem;
        this.gyro = null;
        PIDSetup();

        addRequirements(driveSubsystem);
    }
    public AlignCommand(int turnToAngle, Object gyro, DriveSubsystem driveSubsystem) {
        // turnToAngle is absolute (direction robot faces at the start is 0)
        limelight = null;
        useLimelight = false;
        this.driveSubsystem = driveSubsystem;
        this.gyro = gyro;
        this.turnToAngle = turnToAngle;

        PIDSetup();

        addRequirements(driveSubsystem);
    }

    private void PIDSetup() {
        NetworkTableInstance.getDefault()
                .getTable("alignPid")
                .getEntry("kp")
                .addListener(
                        (notification) -> {
                            Kp = notification.value.getDouble();
                        },
                        EntryListenerFlags.kUpdate & EntryListenerFlags.kNew & EntryListenerFlags.kImmediate);

        // Degrees, degrees / second
        aimyAim.setTolerance(3, 0.5);
        aimyAim.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
        // turning left is negative, right is positive
        double offsetAngle;
        if (useLimelight)
            offsetAngle = limelight.getYaw();
        else {
            // use gyroscope
            double currentAngle = gyro.getAngle();
            offsetAngle = turnToAngle - currentAngle;
        }
        
        double output = aimyAim.calculate(offsetAngle, 0);

        driveSubsystem.drive(0, output);
    }

    @Override
    public boolean isFinished() {
        return aimyAim.atSetpoint();
    }
}
