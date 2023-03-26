package frc.robot.led.animations.brightness;

import frc.robot.led.LEDStrip;
import frc.robot.led.animations.base.BrightnessAnimation;

public class SetBrightness extends BrightnessAnimation {

    private final LEDStrip ledStrip;

    private final double brightness;

    public SetBrightness(double brightness, LEDStrip ledStrip) {
        super(ledStrip);
        this.ledStrip = ledStrip;
        this.brightness = brightness;
    }

    @Override
    public void run() {
        ledStrip.setBrightness(brightness);
    }
    
}
