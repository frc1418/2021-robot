package frc.robot.commands;

// import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.common.Limelight;
import frc.robot.subsystems.DriveSubsystem;

public abstract class AlignCommand extends CommandBase {

    private static final double Ki = 0.00200;
    private static final double Kd = 0.000004;
    private static double Kp = 0.01700;

    protected final PIDController pid = new PIDController(Kp, Ki, Kd);
    protected final DriveSubsystem driveSubsystem;

    public AlignCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        pidSetup();
        addRequirements(driveSubsystem);
    }

    private void pidSetup() {
        SmartDashboard.putData("alignPID", pid);

        // Degrees, degrees / second
        pid.setTolerance(3.5, 0.1);
        pid.setIntegratorRange(-4, 4);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}
