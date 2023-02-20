package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CompressorConstants;

public class CompressorSys extends SubsystemBase {

    // Declare actuators, sensors, and other variables here
    private final Compressor compressor;

    /**
     * Constructs a new CompressorSys.
     * 
     * <p>CompressorSys is used to control the operation of the air compressor.
     */
    public CompressorSys() {
        // Initialize and configure actuators and sensors here
        compressor = new Compressor(PneumaticsModuleType.REVPH);

        compressor.enableAnalog(CompressorConstants.minPressurePSI, CompressorConstants.maxPressurePSI);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        if(RobotController.isBrownedOut())
            compressor.disable();

        SmartDashboard.putNumber("pressure PSI", compressor.getAnalogVoltage());
    }

    // Put methods for controlling this subsystem here. Call these from Commands.
    public boolean isEnabled() {
        return compressor.isEnabled();
    }

    public double getPressurePSI() {
        return compressor.getPressure();
    }

}