package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class AutomaticShootCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final double targetVel;
    private final int ballNum;
    private int totalShoot = 0;

    public AutomaticShootCommand(double targetVel, int ballNum, ShooterSubsystem shooterSubsystem) {
        this.targetVel = targetVel;
        this.ballNum = ballNum;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    public boolean isFinished() {
        return totalShoot >= ballNum;
    }

    public void execute() {
        shooterSubsystem.shootVelocity(targetVel);
        if (shooterSubsystem.checkTarget() && shooterSubsystem.isBallReady()) {
            shooterSubsystem.activatePiston();
            totalShoot++;
        }
        else if (shooterSubsystem.checkTarget() && !shooterSubsystem.isBallReady())
            shooterSubsystem.lowerPiston();
    }
}
