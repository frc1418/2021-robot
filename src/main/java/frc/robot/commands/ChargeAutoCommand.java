package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ChargeAutoCommand extends WaitCommand {

    private final DriveSubsystem driveSubsystem;
    private final double speed;

    public ChargeAutoCommand(DriveSubsystem drive, double speed, double timeout) {
        super(timeout);
        driveSubsystem = drive;
        this.speed = speed;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.drive(this.speed, 0);
    }
}
