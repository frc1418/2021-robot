package frc.robot.commands;

import frc.robot.common.Limelight;
import frc.robot.subsystems.DriveSubsystem;

public class AlignWithLimelightCommand extends AlignCommand {

    private final Limelight limelight;

    public AlignWithLimelightCommand(Limelight limelight, DriveSubsystem driveSubsystem) {
        super(driveSubsystem);
        this.limelight = limelight;
    }

    @Override
    public void execute() {
        double output = this.pid.calculate(limelight.getYaw());
        this.driveSubsystem.drive(0, -output);
    }
}
