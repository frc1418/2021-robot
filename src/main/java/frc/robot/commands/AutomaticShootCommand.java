package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ShooterSubsystem;

public class AutomaticShootCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final Trigger ballSensorTrigger;
    private final double targetVel;
    private int ballsLeftToShoot;

    public AutomaticShootCommand(double targetVel, int ballsLeft, ShooterSubsystem shooterSubsystem) {
        this.targetVel = targetVel;
        this.ballsLeftToShoot = ballsLeft;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
        ballSensorTrigger = new Trigger(this.shooterSubsystem::isBallReady);
        ballSensorTrigger.whenActive(
                new InstantCommand(
                        () -> {
                            ballsLeftToShoot--;
                            // logger.info("Total Balls Shot: " + totalShoot);
                        }));
    }

    public void end(boolean interrupted) {
        shooterSubsystem.shootVoltage(0);
        shooterSubsystem.lowerPiston();
    }

    public boolean isFinished() {
        return (ballsLeftToShoot == 0);
    }

    public void execute() {
        shooterSubsystem.shootVelocity(targetVel);
        if (shooterSubsystem.isBallReady()) {
            shooterSubsystem.activatePiston();
        } else if (!shooterSubsystem.isBallReady()) shooterSubsystem.lowerPiston();
    }
}
