package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.CompressorConstants;

public class CompressorSys extends SubsystemBase {

    private final Compressor compressor;

    /**
     * Constructs a new CompressorSys.
     * 
     * <p>CompressorSys is used to control the operation of the air compressor.
     */
    public CompressorSys() {
        compressor = new Compressor(CANDevices.pneumaticHubId, PneumaticsModuleType.REVPH);

        compressor.enableAnalog(CompressorConstants.minPressurePSI, CompressorConstants.maxPressurePSI);

    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        if(RobotController.isBrownedOut())
            compressor.disable();

        SmartDashboard.putNumber("pressure PSI", compressor.getPressure());

        if(compressor.getPressure() < 0.0)
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
     * Returns the pressure of the air tanks.
     * 
     * @return The pressure of the air tanks, in PSI.
     */
    public double getPressurePSI() {
        return compressor.getPressure();
    }

}