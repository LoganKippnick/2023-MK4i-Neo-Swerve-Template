package frc.robot.led.animations.base;

import java.util.function.BooleanSupplier;

import frc.robot.led.LEDStrip;

public abstract class ColorAnimation extends LEDAnimation {

    public ColorAnimation(LEDStrip ledStrip) {
        super(ledStrip);
    }

    public ColorAnimation(LEDStrip ledStrip, BooleanSupplier cycleCondition) {
        super(ledStrip, cycleCondition);
    }

    public CombinedAnimation alongWith(BrightnessAnimation brightnessAnimation) {
        return new CombinedAnimation(this, brightnessAnimation);
    }
}
