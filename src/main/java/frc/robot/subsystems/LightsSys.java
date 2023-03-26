package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GameElement;
import frc.robot.led.LEDStrip;

public class LightsSys extends SubsystemBase {

    LEDStrip ledStrip;

    AddressableLED led;
    AddressableLEDBuffer buffer;

    GameElement status = GameElement.kNone;

    /**
     * Constructs a new ExampleSys.
     * 
     * <p>ExampleSys contains the basic framework of a robot subsystem for use in command-based programming.
     */
    public LightsSys() {
        ledStrip = new LEDStrip(8, 65);
        ledStrip.setDimness(0.25);
        
        setStatus(GameElement.kNone);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        
    }

    public GameElement getStatus() {
        return status;
    }

    public void setStatus(GameElement status) {
        if(status.equals(GameElement.kCone)) {
            ledStrip.setColor(Color.kYellow);
        }
        else if(status.equals(GameElement.kCube)) {
            ledStrip.setColor(Color.kPurple);
        }
        else {
            ledStrip.setColor(Color.kLime);
        }
        ledStrip.setBrightness(1.0);
    }

    // Put methods for controlling this subsystem here. Call these from Commands.

}