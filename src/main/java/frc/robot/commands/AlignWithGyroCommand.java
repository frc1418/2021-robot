package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.subsystems.DriveSubsystem;

public class AlignWithGyroCommand extends AlignCommand {
    private final AHRS navx;
    private final int turnToAngle;

    public AlignWithGyroCommand(AHRS navx, DriveSubsystem driveSubsystem, int turnToAngle) {
        super(driveSubsystem);
        this.navx = navx;
        this.turnToAngle = turnToAngle;
    }

    @Override
    public void execute() {
        double output = this.pid.calculate(navx.getAngle(), turnToAngle);
        this.driveSubsystem.drive(0, -output);
    }
}
