package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class AlignWithLimelightCommand extends AlignCommand {

    private final Limelight limelight;

    public AlignWithLimelightCommand(Limelight limelight, DriveSubsystem driveSubsystem) {
        super(driveSubsystem);
        this.limelight = limelight;

        pid.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
        double output = this.pid.calculate(limelight.getYaw());
        this.driveSubsystem.drive(0, -output);
    }
}
