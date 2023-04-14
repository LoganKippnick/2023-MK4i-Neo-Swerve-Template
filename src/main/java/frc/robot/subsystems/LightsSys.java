package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GameElement;
import frc.robot.led.LEDStrip;
import frc.robot.led.animations.AnimationDirection;

public class LightsSys extends SubsystemBase {

    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private final LEDStrip rightStrip;
    private final LEDStrip leftStrip;
    private final LEDStrip outerLeftStrip;
    private final LEDStrip outerRightStrip;
    private final LEDStrip innerRightStrip;
    private final LEDStrip innerLeftStrip;

    private final int hopperStripsOffset = 14;

    private GameElement status = GameElement.kNone;

    private final Color neutralColor = new Color(50, 255, 0);
    private final Color coneColor = new Color(150, 128, 0);
    private final Color cubeColor = new Color(204, 0, 255);

    private final double dimness = 1.0;

    private boolean isBlinking = false;
    private int blinkCounter = 0;
    private final Timer blinkTimer;

    private final double blinkTime = 0.125;
    private final int blinkCount = 3;

    private boolean isPartyMode = false;
    public boolean isPartyMode() {
        return isPartyMode;
    }

    private int partyHue;
    private final int partyHueIncrement = 5;

    private boolean isWeeWooMode = false;
    public boolean isWeeWooMode() {
        return isWeeWooMode;
    }
    private final double weeWooBlinkTime = 0.25;
    private final double weeWooHopperBlinkTime = 0.15;
    private final double weeWooOffTime = 0.05;

    private final Timer weeWooTimer;
    private final Timer weeWooHopperTimer;

    private boolean isIntaking = false;
    public boolean isIntaking() {
        return isIntaking;
    }
    
    private double intakeMarqueeBrightness;
    private final int intakeMarqueeLength = 30;

    /**
     * Constructs a new ExampleSys.
     * 
     * <p>ExampleSys contains the basic framework of a robot subsystem for use in command-based programming.
     */
    public LightsSys() {

        rightStrip = new LEDStrip(64);
        rightStrip.setDimness(dimness);

        leftStrip = new LEDStrip(64);
        leftStrip.setDimness(dimness);

        outerLeftStrip = new LEDStrip(32);
        outerLeftStrip.setDimness(dimness);

        outerRightStrip = new LEDStrip(32);
        outerRightStrip.setDimness(dimness);

        innerRightStrip = new LEDStrip(22);
        innerRightStrip.setDimness(dimness);

        innerLeftStrip = new LEDStrip(22);
        innerLeftStrip.setDimness(dimness);

        led = new AddressableLED(8);

        buffer = new AddressableLEDBuffer(
            rightStrip.getLength() +
            leftStrip.getLength() +
            outerLeftStrip.getLength() +
            outerRightStrip.getLength() +
            innerRightStrip.getLength() +
            innerLeftStrip.getLength()
        );
        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();

        blinkTimer = new Timer();
        weeWooTimer = new Timer();
        weeWooHopperTimer = new Timer();
        
        setStatus(GameElement.kNone);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {

        if((!isPartyMode && !isWeeWooMode) || isBlinking) {
            if(status.equals(GameElement.kCone)) {
                rightStrip.setColor(coneColor);
                leftStrip.setColor(coneColor);
                outerLeftStrip.setColor(coneColor);
                outerRightStrip.setColor(coneColor);
                innerRightStrip.setColor(coneColor);
                innerLeftStrip.setColor(coneColor);
            }
            else if(status.equals(GameElement.kCube)) {
                rightStrip.setColor(cubeColor);
                leftStrip.setColor(cubeColor);
                outerLeftStrip.setColor(cubeColor);
                outerRightStrip.setColor(cubeColor);
                innerRightStrip.setColor(cubeColor);
                innerLeftStrip.setColor(cubeColor);
            }
            else {
                rightStrip.setColor(neutralColor);
                leftStrip.setColor(neutralColor);
                outerLeftStrip.setColor(neutralColor);
                outerRightStrip.setColor(neutralColor);
                innerRightStrip.setColor(neutralColor);
                innerLeftStrip.setColor(neutralColor);
            }
        }

        if(isBlinking) {
            blinkTimer.start();
            if(blinkCounter >= blinkCount) {
                isBlinking = false;
            }
            else if(blinkTimer.hasElapsed(blinkTime * 2)) {
                rightStrip.setBrightness(1.0);
                leftStrip.setBrightness(1.0);
                outerLeftStrip.setBrightness(1.0);
                outerRightStrip.setBrightness(1.0);
                innerRightStrip.setBrightness(1.0);
                innerLeftStrip.setBrightness(1.0);

                blinkCounter++;
                blinkTimer.reset();
            }
            else if(blinkTimer.hasElapsed(blinkTime)) {
                rightStrip.setBrightness(0.0);
                leftStrip.setBrightness(0.0);
                outerLeftStrip.setBrightness(0.0);
                outerRightStrip.setBrightness(0.0);
                innerRightStrip.setBrightness(0.0);
                innerLeftStrip.setBrightness(0.0);
            }
            else if(blinkTimer.get() == 0.0 && blinkCounter == 0) {
                rightStrip.setBrightness(1.0);
                leftStrip.setBrightness(1.0);
                outerLeftStrip.setBrightness(1.0);
                outerRightStrip.setBrightness(1.0);
                innerRightStrip.setBrightness(1.0);
                innerLeftStrip.setBrightness(1.0);
            }
        }
        else if(isPartyMode) {
            for(int i = 0; i < rightStrip.getLength(); i++) {
                rightStrip.setColor(Color.fromHSV((partyHue + i) % 180, 255, 255), i);
            }
            for(int i = leftStrip.getLength() - 1; i >= 0; i--) {
                leftStrip.setColor(Color.fromHSV((partyHue + leftStrip.getLength() - 1 - i) % 180, 255, 255), i);
            }
            for(int i = 0; i < outerLeftStrip.getLength(); i++) {
                outerLeftStrip.setColor(Color.fromHSV((partyHue + i) % 180, 255, 255), i);
            }
            for(int i = outerRightStrip.getLength() - 1; i >= 0; i--) {
                outerRightStrip.setColor(Color.fromHSV((partyHue + outerRightStrip.getLength() - 1 - i) % 180, 255, 255), i);
            }
            for(int i = innerRightStrip.getLength() - 1; i >= 0; i--) {
                innerRightStrip.setColor(Color.fromHSV((partyHue + innerRightStrip.getLength() - 1 - i) % 180, 255, 255), i);
            }
            for(int i = 0; i < innerLeftStrip.getLength(); i++) {
                innerLeftStrip.setColor(Color.fromHSV((partyHue + i) % 180, 255, 255), i);
            }

            partyHue -= partyHueIncrement;
            if(partyHue < 0) {
                partyHue = 180;
            }
        }
        else if(isWeeWooMode) {
            weeWooTimer.start();

            if(weeWooTimer.get() < weeWooBlinkTime) {
                rightStrip.setColor(Color.kRed);
                leftStrip.setColor(Color.kBlue);
            }
            else if(weeWooTimer.get() < weeWooBlinkTime + weeWooOffTime) {
                rightStrip.setColor(Color.kBlack);
                leftStrip.setColor(Color.kBlack);
            }
            else if(weeWooTimer.get() < weeWooBlinkTime + weeWooOffTime + weeWooBlinkTime) {
                rightStrip.setColor(Color.kRed);
                leftStrip.setColor(Color.kBlue);
            }
            else if(weeWooTimer.get() < 2 * (weeWooBlinkTime + weeWooOffTime)) {
                rightStrip.setColor(Color.kBlack);
                leftStrip.setColor(Color.kBlack);
            }
            else if(weeWooTimer.get() < 2 * (weeWooBlinkTime + weeWooOffTime) + weeWooBlinkTime) {
                rightStrip.setColor(Color.kBlue);
                leftStrip.setColor(Color.kRed);
            }
            else if(weeWooTimer.get() < 3 * (weeWooBlinkTime + weeWooOffTime)) {
                rightStrip.setColor(Color.kBlack);
                leftStrip.setColor(Color.kBlack);
            }
            else if(weeWooTimer.get() < 3 * (weeWooBlinkTime + weeWooOffTime) + weeWooBlinkTime) {
                rightStrip.setColor(Color.kBlue);
                leftStrip.setColor(Color.kRed);
            }
            else if(weeWooTimer.get() < 4 * (weeWooBlinkTime + weeWooOffTime)) {
                rightStrip.setColor(Color.kBlack);
                leftStrip.setColor(Color.kBlack);
            }
            else {
                weeWooTimer.reset();
            }

            weeWooHopperTimer.start();

            if(weeWooHopperTimer.get() < weeWooHopperBlinkTime) {
                outerRightStrip.setColor(Color.kRed);
                innerRightStrip.setColor(Color.kRed);
                outerLeftStrip.setColor(Color.kBlue);
                innerLeftStrip.setColor(Color.kBlue);
            }
            else if(weeWooHopperTimer.get() < weeWooHopperBlinkTime + weeWooOffTime) {
                outerRightStrip.setColor(Color.kBlack);
                innerRightStrip.setColor(Color.kBlack);
                outerLeftStrip.setColor(Color.kBlack);
                innerLeftStrip.setColor(Color.kBlack);
            }
            else if(weeWooHopperTimer.get() < weeWooHopperBlinkTime + weeWooOffTime + weeWooHopperBlinkTime) {
                outerRightStrip.setColor(Color.kRed);
                innerRightStrip.setColor(Color.kRed);
                outerLeftStrip.setColor(Color.kBlue);
                innerLeftStrip.setColor(Color.kBlue);
            }
            else if(weeWooHopperTimer.get() < 2 * (weeWooHopperBlinkTime + weeWooOffTime)) {
                outerRightStrip.setColor(Color.kBlack);
                innerRightStrip.setColor(Color.kBlack);
                outerLeftStrip.setColor(Color.kBlack);
                innerLeftStrip.setColor(Color.kBlack);
            }
            else if(weeWooHopperTimer.get() < 2 * (weeWooHopperBlinkTime + weeWooOffTime) + weeWooHopperBlinkTime) {
                outerRightStrip.setColor(Color.kBlue);
                innerRightStrip.setColor(Color.kBlue);
                outerLeftStrip.setColor(Color.kRed);
                innerLeftStrip.setColor(Color.kRed);
            }
            else if(weeWooHopperTimer.get() < 3 * (weeWooHopperBlinkTime + weeWooOffTime)) {
                outerRightStrip.setColor(Color.kBlack);
                innerRightStrip.setColor(Color.kBlack);
                outerLeftStrip.setColor(Color.kBlack);
                innerLeftStrip.setColor(Color.kBlack);
            }
            else if(weeWooHopperTimer.get() < 3 * (weeWooHopperBlinkTime + weeWooOffTime) + weeWooHopperBlinkTime) {
                outerRightStrip.setColor(Color.kBlue);
                innerRightStrip.setColor(Color.kBlue);
                outerLeftStrip.setColor(Color.kRed);
                innerLeftStrip.setColor(Color.kRed);
            }
            else if(weeWooHopperTimer.get() < 4 * (weeWooHopperBlinkTime + weeWooOffTime)) {
                outerRightStrip.setColor(Color.kBlack);
                innerRightStrip.setColor(Color.kBlack);
                outerLeftStrip.setColor(Color.kBlack);
                innerLeftStrip.setColor(Color.kBlack);
            }
            else {
                weeWooHopperTimer.reset();
            }
        }
        
        if(!isBlinking) {   
            blinkTimer.stop();
            blinkTimer.reset();

            blinkCounter = 0;

            rightStrip.setBrightness(1.0);
            leftStrip.setBrightness(1.0);
        }

        if(isIntaking) {
            intakeMarqueeBrightness -= 1.0 / intakeMarqueeLength;
            if(intakeMarqueeBrightness < 0.0) intakeMarqueeBrightness = 1.0;

            outerLeftStrip.translateBrightnesses(intakeMarqueeBrightness, AnimationDirection.kReverse);
            outerRightStrip.translateBrightnesses(intakeMarqueeBrightness, AnimationDirection.kForward);

            innerRightStrip.translateBrightnesses(outerRightStrip.getBrightnessState(5), AnimationDirection.kReverse);
            innerLeftStrip.translateBrightnesses(innerLeftStrip.getBrightnessState(innerLeftStrip.getLength() - 4), AnimationDirection.kForward);
        }
        else if(!isBlinking) {
            outerLeftStrip.setBrightness(1.0);
            outerRightStrip.setBrightness(1.0);
            innerRightStrip.setBrightness(1.0);
            innerLeftStrip.setBrightness(1.0);
        }
            

        if(!isPartyMode) {
            partyHue = 0;
        }

        if(!isWeeWooMode) {
            weeWooTimer.stop();
            weeWooTimer.reset();
        }

        update();
    }

    public GameElement getStatus() {
        return status;
    }

    public void setStatus(GameElement status) {
        this.status = status;
    }

    public void blink() {
        isBlinking = true;
    }

    public void setPartyMode(boolean isPartyMode) {
        this.isPartyMode = isPartyMode;
    }

    public void setWeeWooMode(boolean isWeeWooMode) {
        this.isWeeWooMode = isWeeWooMode;
    }

    public void setIntaking(boolean isIntaking) {
        this.isIntaking = isIntaking;
    }

    public void cancelAnimations() {
        isPartyMode = false;
        isWeeWooMode = false;
    }

    public void update() {
        int pixel = 0;
        for(int i = 0; i < rightStrip.getLength(); i++) {
            buffer.setLED(pixel, rightStrip.getState(i));
            pixel++;
        }
        for(int i = 0; i < leftStrip.getLength(); i++) {
            buffer.setLED(pixel, leftStrip.getState(i));
            pixel++;
        }
        for(int i = 0; i < outerLeftStrip.getLength(); i++) {
            buffer.setLED(pixel, outerLeftStrip.getState(i));
            pixel++;
        }
        for(int i = 0; i < outerRightStrip.getLength(); i++) {
            buffer.setLED(pixel, outerRightStrip.getState(i));
            pixel++;
        }
        for(int i = 0; i < innerRightStrip.getLength(); i++) {
            buffer.setLED(pixel, innerRightStrip.getState(i));
            pixel++;
        }
        for(int i = 0; i < innerLeftStrip.getLength(); i++) {
            buffer.setLED(pixel, innerLeftStrip.getState(i));
            pixel++;
        }
        led.setData(buffer);
    }
}