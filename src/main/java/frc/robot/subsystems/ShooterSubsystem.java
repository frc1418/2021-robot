package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax shooterMotor1;
    private final CANSparkMax shooterMotor2;
    private final Solenoid shooterSolenoid;
    private final CANPIDController shooterController;
    private final CANEncoder shooterEncoder;

    public ShooterSubsystem(
            CANSparkMax shooterMotor1,
            CANSparkMax shooterMotor2,
            Solenoid shooterSolenoid,
            CANEncoder encoder) {
        this.shooterMotor1 = shooterMotor1;
        this.shooterMotor2 = shooterMotor2;
        this.shooterSolenoid = shooterSolenoid;
        this.shooterController = shooterMotor1.getPIDController();
        this.shooterEncoder = encoder;

        this.shooterController.setFF(0.0002555);

        shooterMotor1.setInverted(true);
        shooterMotor2.follow(shooterMotor1);
    }

    public void activatePiston() {
        shooterSolenoid.set(true);
    }

    public void lowerPiston() {
        shooterSolenoid.set(false);
    }

    public void shoot(double shooterSpeed) {
        shooterController.setReference(2000, ControlType.kVelocity);
        // shooterMotor1.set(shooterSpeed);
    }

    public void periodic() {
        System.out.println(this.shooterEncoder.getVelocity());
    }
}
