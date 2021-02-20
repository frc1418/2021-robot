package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ShooterSubsystem;

public class AutomaticShootCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final Trigger ballSensorTrigger;
    private final double targetVel;
    private int numBalls;
    private int totalShoot = 0;

    public AutomaticShootCommand(double targetVel, int numBalls, ShooterSubsystem shooterSubsystem) {
        this.targetVel = targetVel;
        this.numBalls = numBalls;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
        ballSensorTrigger = new Trigger(this.shooterSubsystem::isBallReady);
        ballSensorTrigger.whenActive(
                new InstantCommand(
                        () -> {
                            totalShoot++;
                            // logger.info("Total Balls Shot: " + totalShoot);
                        }));
    }

    public void end(boolean interrupted) {
        shooterSubsystem.shootVoltage(0);
    }

    public boolean isFinished() {
        return (totalShoot >= numBalls);
    }

    public void execute() {
        shooterSubsystem.shootVelocity(targetVel);
        if (shooterSubsystem.isAtTargetSpeed() && shooterSubsystem.isBallReady()) {
            shooterSubsystem.activatePiston();
        } else if (shooterSubsystem.isAtTargetSpeed() && !shooterSubsystem.isBallReady())
            shooterSubsystem.lowerPiston();
    }
}
