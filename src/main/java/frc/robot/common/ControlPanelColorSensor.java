package frc.robot.common;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;

public class ControlPanelColorSensor {

    private final ColorMatch colorMatcher;
    private final ColorSensorV3 colorSensor;

    public ControlPanelColorSensor(ColorMatch colorMatcher, ColorSensorV3 colorSensor) {
        this.colorMatcher = colorMatcher;
        this.colorSensor = colorSensor;
        for (ControlPanelColor matchableColor : ControlPanelColor.getValidColors()) {
            colorMatcher.addColorMatch(matchableColor.getColor());
        }
    }

    public ControlPanelColor getDetectedColor() {
        Color rawColor = colorSensor.getColor();
        return ControlPanelColor.valueOf(colorMatcher.matchClosestColor(rawColor).color);
    }
}
