package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.logging.Logger;

public class IntakeSubsystem extends SubsystemBase {
    private static final double BALL_JAMMED_CURRENT = 20;
    private final Logger logger = Logger.getLogger("IntakeSubsystem");
    private final WPI_VictorSPX upperIntakeMotor;
    private final CANSparkMax bottomIntakeMotor;
    private final DigitalInput intakeSwitch;
    private final DoubleSolenoid intakeSolenoid;
    private final Trigger intakeSwitchButton;
    private final boolean isAlreadyPushed = false;
    private boolean jammed = false;
    private int ballsCollected;

    public IntakeSubsystem(
            WPI_VictorSPX upperIntakeMotor,
            CANSparkMax bottomIntakeMotor,
            DigitalInput intakeSwitch,
            DoubleSolenoid intakeSolenoid) {
        this.upperIntakeMotor = upperIntakeMotor;
        this.bottomIntakeMotor = bottomIntakeMotor;
        this.intakeSwitch = intakeSwitch;
        this.intakeSolenoid = intakeSolenoid;

        bottomIntakeMotor.setIdleMode(IdleMode.kCoast);
        bottomIntakeMotor.setOpenLoopRampRate(1.5);
        // upperIntakeMotor.setInverted(InvertType.OpposeMaster);
        intakeSwitchButton = new Trigger(intakeSwitch::get);
        intakeSwitchButton.whenActive(
                new InstantCommand(
                        () -> {
                            ballsCollected++;
                            logger.info("Ball count: " + ballsCollected);
                        }));
    }

    // Solenoid methods

    public void extend() {
        intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void retract() {
        intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void spin(double upperMotorVolts, double bottomMotorVolts) {
        spin(upperMotorVolts, bottomMotorVolts, true);
    }

    public void spin(double upperMotorVolts, double bottomMotorVolts, boolean jamOverride) {
        if (jamOverride && jammed && bottomMotorVolts != 0) {
            upperIntakeMotor.setVoltage(0);
            bottomIntakeMotor.setVoltage(6);
            return;
        }

        upperIntakeMotor.setVoltage(upperMotorVolts);
        bottomIntakeMotor.setVoltage(bottomMotorVolts);
    }

    @Override
    public void periodic() {
        if (bottomIntakeMotor.getOutputCurrent() > BALL_JAMMED_CURRENT) {
            jammed = true;
        } else {
            jammed = false;
        }
    }
}
