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
    // Aka "Amy"
    private final PIDController aimyAim = new PIDController(Kp, Ki, Kd);
    private final Limelight limelight;
    private final DriveSubsystem driveSubsystem;

    public AlignCommand(Limelight limelight, DriveSubsystem driveSubsystem) {
        this.limelight = limelight;
        this.driveSubsystem = driveSubsystem;

        NetworkTableInstance.getDefault()
            .getTable("alignPid")
            .getEntry("kp")
            .addListener((notification) -> {
                Kp = notification.value.getDouble();
            }, EntryListenerFlags.kUpdate & EntryListenerFlags.kNew
                & EntryListenerFlags.kImmediate);

        // Degrees, degrees / second
        aimyAim.setTolerance(3, 0.5);
        aimyAim.enableContinuousInput(-180, 180);

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double output = aimyAim.calculate(limelight.getYaw(), 0);

        driveSubsystem.drive(0, output);
    }

    @Override
    public boolean isFinished() {
        return aimyAim.atSetpoint();
    }
}
