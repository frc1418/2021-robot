package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ShooterSubsystem;

public class AutomaticShootCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final double targetVel;
    private int ballsLeftToShoot;
    private long timeOfLastShot = 0;

    public AutomaticShootCommand(double targetVel, int ballsLeft, ShooterSubsystem shooterSubsystem) {
        this.targetVel = targetVel;
        this.ballsLeftToShoot = ballsLeft;
        this.shooterSubsystem = shooterSubsystem;

        Trigger ballSensor = new Trigger(shooterSubsystem::isBallReady);
        ballSensor.whenInactive(new InstantCommand(() -> ballsLeftToShoot--));
        addRequirements(shooterSubsystem);
    }

    public void end(boolean interrupted) {
        // shooterSubsystem.shootVoltage(0);
        shooterSubsystem.lowerPiston();
    }

    public boolean isFinished() {
        return ballsLeftToShoot <= 0;
    }

    public void execute() {
        if (shooterSubsystem.isBallReady()) {
            shooterSubsystem.activatePiston();
        } else if (!shooterSubsystem.isBallReady()) {
            shooterSubsystem.lowerPiston();
        }
    }
}
