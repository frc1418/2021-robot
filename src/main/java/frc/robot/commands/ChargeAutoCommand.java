package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ChargeAutoCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final double speed;

    public ChargeAutoCommand(DriveSubsystem drive, double speed) {
        driveSubsystem = drive;
        this.speed = speed;
    }

    @Override
    public void execute() {
        driveSubsystem.drive(this.speed, 0);
    }
}
