package frc.robot.led.animations.base;

import java.util.function.BooleanSupplier;

import frc.robot.led.LEDStrip;

public abstract class BrightnessAnimation extends LEDAnimation {

    public BrightnessAnimation(LEDStrip ledStrip) {
        super(ledStrip);
    }

    public BrightnessAnimation(LEDStrip ledStrip, BooleanSupplier cycleCondition) {
        super(ledStrip, cycleCondition);
    }
    
    public CombinedAnimation alongWith(ColorAnimation colorAnimation) {
        return new CombinedAnimation(colorAnimation, this);
    }
}
