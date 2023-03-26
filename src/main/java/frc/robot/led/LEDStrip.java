package frc.robot.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.led.animations.AnimationDirection;
import frc.robot.led.animations.SetSolid;
import frc.robot.led.animations.base.BrightnessAnimation;
import frc.robot.led.animations.base.ColorAnimation;
import frc.robot.led.animations.base.CombinedAnimation;

public class LEDStrip {

    private final AddressableLED led;

    private final AddressableLEDBuffer buffer;
    public AddressableLEDBuffer getState() {
        return buffer;
    }

    private final AddressableLEDBuffer offBuffer;

    private final Color[] colorStates;
    private final double[] brightnessStates;

    private double dimness = 1.0;
    public double getDimness() {
        return dimness;
    }
    public void setDimness(double dimness) {
        this.dimness = dimness;
    }

    private CombinedAnimation defaultAnimation;
    private CombinedAnimation currentAnimation;

    private boolean isEnabled = true;
    public boolean isEnabled() {
        return isEnabled;
    }
    public void setEnabled(boolean isEnabled) {
        this.isEnabled = isEnabled;
    }
    
    public LEDStrip(int pwmPort, int length) {
        led = new AddressableLED(pwmPort);
        led.setLength(length);

        buffer = new AddressableLEDBuffer(length);

        offBuffer = new AddressableLEDBuffer(buffer.getLength());
        for(int i = 0; i < offBuffer.getLength(); i++) offBuffer.setLED(i, Color.kBlack);

        colorStates = new Color[length];
        for(Color color : colorStates) {
            color = Color.kBlack;
        }

        brightnessStates = new double[length];
        for(double brightness : brightnessStates) {
            brightness = 1.0;
        }

        led.setData(buffer);
        led.start();

        defaultAnimation = new SetSolid(Color.kBlack, this); // TODO: make sure this doesn't have a stroke by calling itself in the constructor
        setAnimation(defaultAnimation);
        defaultAnimation.schedule();
    }

    public void setDefaultAnimation(CombinedAnimation animation) {
        defaultAnimation = animation;
        animation.schedule();
    }

    public void setDefaultAnimation(ColorAnimation animation) {
        defaultAnimation = new CombinedAnimation(animation);
    }

    public void setDefaultAnimation(BrightnessAnimation animation) {
        defaultAnimation = new CombinedAnimation(animation);
    }

    public void setAnimation(CombinedAnimation animation) {
        if(currentAnimation != null)
            currentAnimation.cancel();
        animation.schedule();
        currentAnimation = animation;
    }

    public void setAnimation(ColorAnimation animation) {
        setAnimation(new CombinedAnimation(animation));
    }
    
    public void setAnimation(BrightnessAnimation animation) {
        setAnimation(new CombinedAnimation(animation));
    }

    public void overrideAnimation(ColorAnimation colorAnimation) {
        setAnimation(new CombinedAnimation(colorAnimation, currentAnimation.brightnessAnimation));
    }

    public void overrideAnimation(BrightnessAnimation brightnessAnimation) {
        setAnimation(new CombinedAnimation(currentAnimation.colorAnimation, brightnessAnimation));
    }
    
    public void cancelAnimation() {
        setAnimation(defaultAnimation);
    }

    public Color getPixel(int index) {
        return buffer.getLED(index);
    }

    public void update() {
        if(!isEnabled) {
            led.setData(offBuffer);
        }
        else {
            // Combine color and brightness arrays into buffer and write it to the LEDs
            for(int i = 0; i < buffer.getLength(); i++) {
                buffer.setRGB(
                    i, 
                    (int) Math.round(colorStates[i].red * 255 * brightnessStates[i] * dimness),
                    (int) Math.round(colorStates[i].green * 255 * brightnessStates[i] * dimness),
                    (int) Math.round(colorStates[i].blue * 255 * brightnessStates[i] * dimness)
                );
            }
        }
        led.setData(buffer);
    }

    public Color getColorState(int index) {
        return colorStates[index];
    }

    public void setColor(Color color) {
        // Set LEDs to solid color
        for(int i = 0; i < buffer.getLength(); i++) {
            colorStates[i] = color;
        }
        update();
    }

    public void setColor(Color color, int index) {
        // Set color of specific LED
        colorStates[index] = color;
        update();
    }

    public void translateColors(Color color, AnimationDirection direction) {
        // Translate colors array and set the "empty" pixel to the new color
        if(direction.equals(AnimationDirection.kForward)) {
            for(int i = buffer.getLength() - 1; i >= 0; i--) {
                colorStates[i] = colorStates[i - 1];
            }
            colorStates[0] = color;
        }
        else {
            for(int i = 0; i > buffer.getLength() - 1; i++) {
                colorStates[i] = colorStates[i + 1];
            }
            colorStates[buffer.getLength() - 1] = color;
        }
        update();
    }

    public double getBrightnessState(int index) {
        return brightnessStates[index];
    }

    public void setBrightness(double brightness) {
        if(brightness > 1.0) brightness = 1.0;
        else if(brightness < 0.0) brightness = 0.0;
        // Set LEDs to solid brightness
        for(int i = 0; i < buffer.getLength(); i++) {
            brightnessStates[i] = brightness;
        }
        update();
    }

    public void setBrightness(double brightness, int index) {
        if(brightness > 1.0) brightness = 1.0;
        else if(brightness < 0.0) brightness = 0.0;
        // Set brightness of specific LED
        brightnessStates[index] = brightness;
        update();
    }

    public void translateBrightnesses(double brightness, AnimationDirection direction) {
        // Translate brightnesses array and set the "empty" pixel to the new brightness
        if(direction.equals(AnimationDirection.kForward)) {
            for(int i = buffer.getLength() - 1; i >= 0; i--) {
                brightnessStates[i] = brightnessStates[i - 1];
            }
            brightnessStates[0] = brightness;
        }
        else {
            for(int i = 0; i > buffer.getLength() - 1; i++) {
                brightnessStates[i] = brightnessStates[i + 1];
            }
            brightnessStates[buffer.getLength() - 1] = brightness;
        }
        update();
    }

}
