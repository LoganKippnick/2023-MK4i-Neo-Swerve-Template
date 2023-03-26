package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GameElement;
import frc.robot.led.LEDStrip;

public class LightsSys extends SubsystemBase {

    private final LEDStrip rightStrip;
    private final LEDStrip leftStrip;

    private GameElement status = GameElement.kNone;

    private boolean isBlinking = false;
    private int blinkCounter = 0;
    private final Timer blinkTimer;

    private boolean isPartyMode = false;
    private int partyHue;
    private final int partyHueIncrement = 1;

    private boolean isWeeWooMode = false;
    private final Timer weeWooTimer;

    /**
     * Constructs a new ExampleSys.
     * 
     * <p>ExampleSys contains the basic framework of a robot subsystem for use in command-based programming.
     */
    public LightsSys() {
        rightStrip = new LEDStrip(8, 65);
        rightStrip.setDimness(0.25);

        leftStrip = new LEDStrip(8, 65);
        leftStrip.setDimness(0.25);

        blinkTimer = new Timer();
        weeWooTimer = new Timer();
        
        setStatus(GameElement.kNone);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        if((!isPartyMode && !isWeeWooMode) || isBlinking) {
            if(status.equals(GameElement.kCone)) {
                rightStrip.setColor(Color.kYellow);
                leftStrip.setColor(Color.kYellow);
            }
            else if(status.equals(GameElement.kCube)) {
                rightStrip.setColor(Color.kPurple);
                leftStrip.setColor(Color.kPurple);
            }
            else {
                rightStrip.setColor(Color.kLime);
                leftStrip.setColor(Color.kLime);
            }
        }

        if(isBlinking) {
            if(blinkCounter >= 3) {
                isBlinking = false;
            }
            else if(blinkTimer.hasElapsed(1.0)) {
                rightStrip.setBrightness(1.0);
                leftStrip.setBrightness(1.0);
                blinkCounter++;
                blinkTimer.reset();
            }
            else if(blinkTimer.hasElapsed(0.5)) {
                rightStrip.setBrightness(0.0);
                leftStrip.setBrightness(0.0);
            }
            else if(blinkTimer.get() == 0.0 && blinkCounter == 0) {
                rightStrip.setBrightness(1.0);
                leftStrip.setBrightness(1.0);
                blinkTimer.start();
            }
        }
        else if(isPartyMode) {
            for(int i = 0; i < rightStrip.getLength(); i++) {
                rightStrip.setColor(Color.fromHSV((partyHue + i) % 180, 255, 255), i);
            }
            for(int i = 0; i < leftStrip.getLength(); i++) {
                leftStrip.setColor(Color.fromHSV((partyHue + i) % 180, 255, 255), i);
            }

            partyHue += partyHueIncrement;
            if(partyHue > 180) {
                partyHue -= 180;
            }
        }
        else if(isWeeWooMode) {
            if(weeWooTimer.hasElapsed(1.0)) {
                weeWooTimer.reset();
            }
            else if(weeWooTimer.hasElapsed(0.5)) {
                rightStrip.setColor(Color.kBlue);
                leftStrip.setColor(Color.kBlue);
            }
            else if(weeWooTimer.get() == 0.0) {
                weeWooTimer.start();
                rightStrip.setColor(Color.kRed);
                leftStrip.setColor(Color.kRed);
            }
            else {
                rightStrip.setColor(Color.kRed);
                leftStrip.setColor(Color.kRed);
            }
        }
        
        if(!isBlinking) {   
            blinkTimer.stop();
            blinkTimer.reset();

            blinkCounter = 0;

            rightStrip.setBrightness(1.0);
            leftStrip.setBrightness(1.0);
        }

        if(!isPartyMode) {
            partyHue = 0;
        }

        if(!isWeeWooMode) {
            weeWooTimer.stop();
            blinkTimer.reset();
        }
    }

    public GameElement getStatus() {
        return status;
    }

    public void setStatus(GameElement status) {
        isBlinking = false;

        this.status = status;
    }

    public void blink() {
        isBlinking = true;
    }
}