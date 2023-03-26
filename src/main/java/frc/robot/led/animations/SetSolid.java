package frc.robot.led.animations;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.led.LEDStrip;
import frc.robot.led.animations.base.CombinedAnimation;
import frc.robot.led.animations.brightness.SetBrightness;
import frc.robot.led.animations.color.SetColor;

public class SetSolid extends CombinedAnimation {

    public SetSolid(Color color, double brightness, LEDStrip ledStrip) {
        super(new SetColor(color, ledStrip), new SetBrightness(brightness, ledStrip));
    }

    public SetSolid(Color color, LEDStrip ledStrip) {
        this(color, 1.0, ledStrip);
    }
    
}
