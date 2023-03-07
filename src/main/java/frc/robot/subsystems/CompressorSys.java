package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.CompressorConstants;

public class CompressorSys extends SubsystemBase {

    private final Compressor compressor;

    private final Timer timeoutTimer;
    private final Timer runTimer;

    private int turnOnCount = 0;
    private boolean hasTurnedOff = true;

    /**
     * Constructs a new CompressorSys.
     * 
     * <p>CompressorSys is used to control the operation of the air compressor.
     */
    public CompressorSys() {
        compressor = new Compressor(CANDevices.pneumaticHubId, PneumaticsModuleType.REVPH);

        compressor.enableAnalog(CompressorConstants.minPressurePSI, CompressorConstants.maxPressurePSI);

        timeoutTimer = new Timer();
        runTimer = new Timer();
        runTimer.reset();
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        if(RobotController.isBrownedOut()) {
            setEnabled(false);
            timeoutTimer.stop();
            timeoutTimer.reset();
        }

        if(!compressor.isEnabled() && timeoutTimer.get() == 0.0 && RobotController.getBatteryVoltage() > CompressorConstants.compressorReenableVoltage) {
            timeoutTimer.start();
        }

        if(timeoutTimer.hasElapsed(CompressorConstants.compressorReenableSeconds)) {
            setEnabled(true);
            timeoutTimer.stop();
            timeoutTimer.reset();
        }

        if(isRunning()) runTimer.start();
        else runTimer.stop();

        SmartDashboard.putNumber("pressure PSI", compressor.getPressure());
        // SmartDashboard.putBoolean("compressor enabled", compressor.isEnabled());

        SmartDashboard.putBoolean("compressor running", isRunning());
        SmartDashboard.putNumber("compressor elapsed", runTimer.get());


        if(compressor.getPressure() <= 0.0)
            DriverStation.reportError("PRESSURE RELEASE VALVE IS OPEN", false);

        if(isRunning() && hasTurnedOff) {
            turnOnCount++;
            hasTurnedOff = false;
        }

        SmartDashboard.putNumber("compressor turn on count", turnOnCount);

        if(!isRunning()) hasTurnedOff = true;

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
     * Sets whether the compressor should enable or disable.
     * 
     * @param isEnabled True if the compressor should enable.
     */
    public void setEnabled(boolean isEnabled) {
        if(isEnabled)
            compressor.enableAnalog(CompressorConstants.minPressurePSI, CompressorConstants.maxPressurePSI);
        else
            compressor.disable();
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
        return compressor.getCurrent() > 0.25;
    }

}