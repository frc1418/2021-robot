package frc.robot.common;

import edu.wpi.first.wpilibj.util.Color;

public enum ControlPanelColor {
    R(new Color(0.325, 0.453, 0.221)),
    Y(new Color(0.269, 0.550, 0.181)),
    G(new Color(0.175, 0.469, 0.356)),
    B(new Color(0.188, 0.503, 0.310)),
    UNKNOWN(new Color(0, 0, 0));

    private final Color color;

    public static ControlPanelColor valueOf(Color fromColor) {
        for (ControlPanelColor possible : values()) {
            if (possible.getColor()
                .equals(fromColor)) {
                return possible;
            }
        }
        return UNKNOWN;
        //        return Arrays.stream(ControlPanelColor.values())
        //        .filter(cpColor -> cpColor.getColor()
        //            .equals(fromColor))
        //        .findFirst()
        //        .orElse(UNKNOWN);
    }

    public static ControlPanelColor[] getValidColors() {
        return new ControlPanelColor[]{R, Y, G, B};
    }

    ControlPanelColor(Color color) {
        this.color = color;
    }

    public Color getColor() {
        return color;
    }
}
