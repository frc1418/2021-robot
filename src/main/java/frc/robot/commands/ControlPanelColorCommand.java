package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class ControlPanelColorCommand extends CommandBase {

    private final ControlPanelSubsystem cpSubsystem;
    private float direction;

    public ControlPanelColorCommand(ControlPanelSubsystem controlPanelSubsystem) {
        this.cpSubsystem = controlPanelSubsystem;
    }

    @Override
    public void initialize() {
        cpSubsystem.setSolenoid(true);
        direction = Math.signum(cpSubsystem.getTurnToColor()
            .ordinal() - cpSubsystem.getDetectedColor()
            .ordinal());
    }

    @Override
    public void end(boolean interrupted) {
        cpSubsystem.setSolenoid(false);
        cpSubsystem.spin(0);
    }

    @Override
    public boolean isFinished() {
        return cpSubsystem.isAtTarget();
    }

    @Override
    public void execute() {
        cpSubsystem.spin(0.5 * direction);
    }
}
