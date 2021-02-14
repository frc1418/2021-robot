package frc.robot.commands;

// import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.common.Limelight;
import frc.robot.subsystems.DriveSubsystem;

public abstract class AlignCommand extends CommandBase {

    private static final double Ki = 0.000300;
    private static final double Kd = 0.000005;
    private static double Kp = 0.016000;

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
        pid.setTolerance(0.1, 0.1);
        pid.enableContinuousInput(-180, 180);
        pid.setIntegratorRange(-3.5, 3.5);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}
