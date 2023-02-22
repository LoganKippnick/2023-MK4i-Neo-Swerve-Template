package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.IntakeConstants;

public class IntakeSys extends SubsystemBase {

    // Declare actuators, sensors, and other variables here

    private final CANSparkMax actuationMtr;
    private final CANSparkMax upperMtr;
    private final CANSparkMax lowerMtr;

    private final RelativeEncoder intakeEnc;

    private final SparkMaxPIDController controller;

    private double targetInches = 0;

    /**
     * Constructs a new IntakeSys.
     * 
     * <p>IntakeSys contains methods for control and actuation of the intake.
     */
    public IntakeSys() {
        // Initialize and configure actuators and sensors here
        actuationMtr = new CANSparkMax(CANDevices.actuationMtrId, MotorType.kBrushed);
        upperMtr = new CANSparkMax(CANDevices.upperMtrId, MotorType.kBrushless);
        lowerMtr = new CANSparkMax(CANDevices.lowerMtrId, MotorType.kBrushless);

        intakeEnc = actuationMtr.getEncoder();
        intakeEnc.setPositionConversionFactor(IntakeConstants.encRevToInches);

        controller = actuationMtr.getPIDController();
        controller.setP(IntakeConstants.kP);
        controller.setD(IntakeConstants.kD);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // controller.setReference(targetInches, ControlType.kPosition);

        SmartDashboard.putNumber("intake inches", intakeEnc.getPosition());
    }

    // Put methods for controlling this subsystem here. Call these from Commands.

    public void setTarget(double inches) {
        targetInches = inches;
    }

    public void setPowers(double upperPower, double lowerPower) {
        upperMtr.set(upperPower);
        lowerMtr.set(lowerPower);
    }

    public void setMetersPerSecond(double metersPerSecond) {
        double power = (metersPerSecond * ((Units.metersToInches(1.0) * 60) / (4 * Math.PI))) / IntakeConstants.freeRPM;
        upperMtr.set(power);
        lowerMtr.set(power);
    }

    public void setRPM(double rpm) {
        double power = rpm / IntakeConstants.freeRPM;
        upperMtr.set(power);
        lowerMtr.set(power);
    }
}