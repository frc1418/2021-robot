package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax shooterMotor1;
    private final CANSparkMax shooterMotor2;
    private final Solenoid shooterSolenoid;
    private final CANPIDController shooterController;

    public ShooterSubsystem(CANSparkMax shooterMotor1,
                            CANSparkMax shooterMotor2,
                            Solenoid shooterSolenoid) {
        this.shooterMotor1 = shooterMotor1;
        this.shooterMotor2 = shooterMotor2;
        this.shooterSolenoid = shooterSolenoid;
        this.shooterController = shooterMotor1.getPIDController();

        shooterMotor1.setInverted(true);
        shooterMotor2.follow(shooterMotor1);
    }

    public void activatePiston() {
        shooterSolenoid.set(true);
    }

    public void lowerPiston() {
        shooterSolenoid.set(false);
    }

    /**
     * @param shooterSpeed Velocity of shooter motors in RPM
     */
    public void shoot(double shooterSpeed) {
        // shooterController.setReference(shooterSpeed, ControlType.kVelocity);
        shooterMotor1.set(shooterSpeed);
    }
}
