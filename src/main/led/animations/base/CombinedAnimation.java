package frc.robot.led.animations.base;

import frc.robot.led.LEDStrip;

public class CombinedAnimation extends LEDAnimation {

    public final ColorAnimation colorAnimation;
    public final BrightnessAnimation brightnessAnimation;

    /**
     * User is responsible for making sure both LEDAnimations write to the same LEDStrip.
     * 
     * @param colorAnimation
     * @param brightnessAnimation
     */
    public CombinedAnimation(ColorAnimation colorAnimation, BrightnessAnimation brightnessAnimation) {
        super(
            colorAnimation.getLEDStrip(),
            colorAnimation.getCyclesRun() > brightnessAnimation.getCyclesRun()
                ? colorAnimation.cycleCondition
                : brightnessAnimation.cycleCondition
        );
        this.colorAnimation = colorAnimation;
        this.brightnessAnimation = brightnessAnimation;
    }

    public CombinedAnimation(ColorAnimation colorAnimation) {
        super(colorAnimation.getLEDStrip(), colorAnimation.cycleCondition);
        this.colorAnimation = colorAnimation;
        this.brightnessAnimation = new NullBrightnessAnimation(colorAnimation.getLEDStrip());
    }

    public CombinedAnimation(BrightnessAnimation brightnessAnimation) {
        super(brightnessAnimation.getLEDStrip(), brightnessAnimation.cycleCondition);
        this.colorAnimation = new NullColorAnimation(brightnessAnimation.getLEDStrip());
        this.brightnessAnimation = brightnessAnimation;
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void run() {
        colorAnimation.run();
        brightnessAnimation.run();
    }

    @Override
    public boolean isFinished() {
        return colorAnimation.isFinished() && brightnessAnimation.isFinished();
    }
    
}

class NullColorAnimation extends ColorAnimation {

    public NullColorAnimation(LEDStrip ledStrip) {
        super(ledStrip, () -> false);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public void run() {}

}

class NullBrightnessAnimation extends BrightnessAnimation {

    public NullBrightnessAnimation(LEDStrip ledStrip) {
        super(ledStrip, () -> false);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public void run() {}

}
