package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.DriveSubsystem;

public class AlignWithGyroCommand extends AlignCommand {
    private static final double Ki = 0.00200;
    private static final double Kd = 0.000007;
    private static double Kp = 0.017900;
    private final AHRS navx;
    private final int turnToAngle;

    public AlignWithGyroCommand(AHRS navx, DriveSubsystem driveSubsystem, int turnToAngle) {
        super(driveSubsystem);
        pid = new PIDController(Kp, Ki, Kd);
        this.navx = navx;
        this.turnToAngle = turnToAngle;

        this.pid.setTolerance(4, 0.15);
    }

    @Override
    public void execute() {
        double output = this.pid.calculate(navx.getAngle(), turnToAngle);
        // System.out.println("navx ang: " + navx.getAngle() + "turnToAngle: " + turnToAngle);
        this.driveSubsystem.drive(0, output);
    }
}
