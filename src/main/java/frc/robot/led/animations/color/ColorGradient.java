package frc.robot.led.animations.color;

import edu.wpi.first.wpilibj.util.Color;

public class ColorGradient {

    public final Color startColor;
    public final Color endColor;

    private final double rIncrement;
    private final double gIncrement;
    private final double bIncrement;

    public ColorGradient(Color startColor, Color endColor, int length) {
        this.startColor = startColor;
        this.endColor = endColor;

        rIncrement = (endColor.red - startColor.red) / length - 1;
        gIncrement = (endColor.green - startColor.green) / length - 1;
        bIncrement = (endColor.blue - startColor.blue) / length - 1;
    }
    
    public Color get(int index) {
        return new Color(
            startColor.red + (rIncrement * index),
            startColor.green + (gIncrement * index),
            startColor.blue + (bIncrement * index)
        );
    }
}
