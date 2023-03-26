package frc.robot.led.animations.base;

import frc.robot.led.LEDStrip;

public class SequentialColorAnimation extends ColorAnimation {

    private ColorAnimation currentAnimation;
    private ColorAnimation[] animations;

    private int currentAnimationIndex;

    private LEDStrip ledStrip;

    public SequentialColorAnimation(LEDStrip ledStrip, ColorAnimation... animations) {
        super(ledStrip);
        
        currentAnimation = animations[0];
        this.animations = animations;

        currentAnimationIndex = 0;

        this.ledStrip = ledStrip;
    }

    @Override
    public void initialize() {
        currentAnimation.initialize();
    }

    @Override
    public void execute() {
        currentAnimation.execute();

        if(currentAnimation.isFinished()) {
            currentAnimationIndex++;
            currentAnimation = animations[currentAnimationIndex];
        }
    }

    @Override
    public void run() {}

    @Override
    public void end(boolean interrupted) {
        ledStrip.cancelAnimation();
    }

    @Override
    public boolean isFinished() {
        return animations[animations.length - 1].isFinished();
    }
    
}
