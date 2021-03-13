package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ShooterSubsystem;

public class AutomaticShootCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private Trigger ballSensorTrigger;
    private final double targetVel;
    private int ballsLeftToShoot;
    private int ballsShot = 0;
    private boolean ballFound = false;
    private boolean readyToFire = true;
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

        ballSensorTrigger = null;
        ballsLeftToShoot = 0;
        System.out.println("AUTOMATIC SHOOT ENDED");
    }

    public boolean isFinished() {
        return (false);
    }

    public void execute() {
        // System.out.println("EXECUTING AUTOMATIC SHOOT COMMAND");
        shooterSubsystem.shootVoltage(targetVel);
        if (System.currentTimeMillis() - timeOfLastShot >= 1100) {

            shooterSubsystem.activatePiston();
            System.out.println("piston activated");
            ballFound = true;
            timeOfLastShot = System.currentTimeMillis();

        } else if (System.currentTimeMillis() - timeOfLastShot > 300) {
            shooterSubsystem.lowerPiston();
        }
    }

    @Override
    public void initialize() {
        super.initialize();
        timeOfLastShot = System.currentTimeMillis();
        System.out.println("AutomaticShootCommand initialized");
    }
}
