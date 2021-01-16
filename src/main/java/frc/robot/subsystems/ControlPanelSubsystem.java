package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.ControlPanelColor;
import frc.robot.common.ControlPanelColorSensor;
import java.util.logging.Logger;

public class ControlPanelSubsystem extends SubsystemBase {

    private final DriverStation ds;
    private final DoubleSolenoid cpSolenoid;
    private final WPI_VictorSPX cpMotor;
    private final ControlPanelColorSensor colorSensor;
    private final Logger logger = Logger.getLogger("ControlPanelSubsystem");
    private ControlPanelColor turnToColor;

    public ControlPanelSubsystem(DoubleSolenoid cpSolenoid,
                                 WPI_VictorSPX cpMotor,
                                 ControlPanelColorSensor colorSensor,
                                 DriverStation ds) {
        this.cpSolenoid = cpSolenoid;
        this.cpMotor = cpMotor;
        this.colorSensor = colorSensor;
        this.ds = ds;
    }

    public void spin(double speed) {
        cpMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setSolenoid(boolean extend) {
        if (extend) {
            cpSolenoid.set(Value.kForward);
        } else {
            cpSolenoid.set(Value.kReverse);
        }
        //        cpSolenoid.set(extend ? Value.kForward : Value.kReverse);
    }

    public ControlPanelColor getDetectedColor() {
        return colorSensor.getDetectedColor();
    }

    public ControlPanelColor getTurnToColor() {
        return turnToColor;
    }

    private ControlPanelColor calculateTurnToColor() {
        ControlPanelColor dsColor = ControlPanelColor.valueOf(ds.getGameSpecificMessage());
        int totalColors = ControlPanelColor.getValidColors().length;
        return ControlPanelColor.getValidColors()[(dsColor.ordinal() + 2) % totalColors];
    }

    @Override
    public void periodic() {
        if (ds.getGameSpecificMessage()
            .isEmpty()) {
            return;
        }
        if (this.turnToColor == ControlPanelColor.UNKNOWN) {
            this.turnToColor = calculateTurnToColor();
        }
    }

    public boolean isAtTarget() {
        return this.getDetectedColor() == this.getTurnToColor();
    }
}
