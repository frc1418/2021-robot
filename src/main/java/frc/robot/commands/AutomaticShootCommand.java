package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
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
        addRequirements(shooterSubsystem);
    }

    public void end(boolean interrupted) {
        shooterSubsystem.shootVoltage(0);
        shooterSubsystem.lowerPiston();
    }

    public boolean isFinished() {
        return ballsLeftToShoot <= 0;
    }

    public void execute() {
        shooterSubsystem.shootVoltage(targetVel);
        if (System.currentTimeMillis() - timeOfLastShot >= 700 && shooterSubsystem.isBallReady()) {

            shooterSubsystem.activatePiston();
            timeOfLastShot = System.currentTimeMillis();
            this.ballsLeftToShoot--;
        } else if (System.currentTimeMillis() - timeOfLastShot > 300) {
            shooterSubsystem.lowerPiston();
        }
    }

    @Override
    public void initialize() {
        super.initialize();
        timeOfLastShot = System.currentTimeMillis();
    }
}
