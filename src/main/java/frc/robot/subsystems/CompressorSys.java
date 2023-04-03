package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.CompressorConstants;

public class CompressorSys extends SubsystemBase {

    private final Compressor compressor;

    private final Timer runTimer;

    private boolean hasTurnedOff = true;

    private int turnOnCount = 0;

    /**
     * Constructs a new CompressorSys.
     * 
     * <p>CompressorSys is used to control the operation of the air compressor.
     */
    public CompressorSys() {
        compressor = new Compressor(CANDevices.pneumaticHubId, PneumaticsModuleType.REVPH);

        compressor.enableAnalog(CompressorConstants.minPressurePSI, CompressorConstants.maxPressurePSI);

        runTimer = new Timer();
        runTimer.reset();
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        if(isRunning()) {
            runTimer.start();

            if(hasTurnedOff) turnOnCount++;

            hasTurnedOff = false;
        }
        else {
            runTimer.stop();
            hasTurnedOff = true;
        }

        if(compressor.getPressure() <= 0.0)
            DriverStation.reportError("PRESSURE RELEASE VALVE IS OPEN", false);
    }

    /**
     * Checks if the compressor is enabled.
     * 
     * @return True if the compressor is enabled.
     */
    public boolean isEnabled() {
        return compressor.isEnabled();
    }

    /**
     * Disables the compressor.
     */
    public void disable() {
        compressor.disable();
    }

    /**
     * Enables the compressor.
     */
    public void enable() {
        compressor.enableAnalog(CompressorConstants.minPressurePSI, CompressorConstants.maxPressurePSI);
    }

    /**
     * Enables the compressor and tops it off.
     */
    public void run() {
        compressor.enableAnalog(CompressorConstants.maxPressurePSI - 1.0, CompressorConstants.maxPressurePSI);
        compressor.enableAnalog(CompressorConstants.minPressurePSI, CompressorConstants.maxPressurePSI);
    }

    /**
     * Returns the pressure of the air tanks.
     * 
     * @return The pressure of the air tanks, in PSI.
     */
    public double getPressurePSI() {
        return compressor.getPressure();
    }

    public boolean isRunning() {
        return compressor.getCurrent() > 0.2;
    }

    public double getRunTimeSeconds() {
        return runTimer.get();
    }

    public int getTurnOnCount() {
        return turnOnCount;
    }

}