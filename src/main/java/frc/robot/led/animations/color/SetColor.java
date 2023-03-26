package frc.robot.led.animations.color;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.led.LEDStrip;
import frc.robot.led.animations.base.ColorAnimation;

public class SetColor extends ColorAnimation {

    private final LEDStrip ledStrip;

    private final Color color;

    public SetColor(Color color, LEDStrip ledStrip) {
        super(ledStrip);
        this.ledStrip = ledStrip;
        this.color = color;
    }

    @Override
    public void run() {
        ledStrip.setColor(color);
    }
    
}
