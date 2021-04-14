package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private final SpeedControllerGroup winchMotors;
    private final DoubleSolenoid scissorSolenoid;
    private final WPI_VictorSPX hookMotor;

    public ClimbSubsystem(
            SpeedControllerGroup winchMotors, DoubleSolenoid scissorSolenoid, WPI_VictorSPX hookMotor) {
        this.winchMotors = winchMotors;
        this.scissorSolenoid = scissorSolenoid;
        this.hookMotor = hookMotor;
    }

    public void setWinchMotors(double speed) {
        winchMotors.set(speed);
    }

    public void setScissorSolenoid(DoubleSolenoid.Value value) {
        scissorSolenoid.set(value);
        ;
    }

    public void setHookMotor(double speed) {
        hookMotor.set(speed);
        ;
    }
}
