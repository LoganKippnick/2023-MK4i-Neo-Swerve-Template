package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.IntakeConstants;

public class IntakeSys extends SubsystemBase {

    // Declare actuators, sensors, and other variables here

    private final CANSparkMax actuationMtr;
    private final CANSparkMax upperMtr;
    @SuppressWarnings("unused")
    private final CANSparkMax lowerMtr;

    private final RelativeEncoder intakeEnc;

    private final SparkMaxPIDController controller;

    private double targetInches = 0;

    private boolean actuationIsManual = false;
    private boolean rollersAreManual = false;

    /**
     * Intake needs to be offset since the encoder can't output negative values.
     * This value is an approximate midpoint between zero and its max value.
     */
    private final double offsetInches = 435.0;

    /**
     * Constructs a new IntakeSys.
     * 
     * <p>IntakeSys contains methods for control and actuation of the intake.
     */
    public IntakeSys() {
        // Initialize and configure actuators and sensors here
        actuationMtr = new CANSparkMax(CANDevices.actuationMtrId, MotorType.kBrushed);
        actuationMtr.setInverted(true);
        actuationMtr.setSmartCurrentLimit(IntakeConstants.actuationCurrentLimitAmps);

        upperMtr = new CANSparkMax(CANDevices.upperMtrId, MotorType.kBrushless);
        upperMtr.setInverted(true);

        lowerMtr = null; // new CANSparkMax(CANDevices.lowerMtrId, MotorType.kBrushless);

        intakeEnc = actuationMtr.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
        intakeEnc.setAverageDepth(64);
        intakeEnc.setPosition(offsetInches);
        intakeEnc.setInverted(false);
        intakeEnc.setPositionConversionFactor(IntakeConstants.encRevToInches);

        controller = actuationMtr.getPIDController();
        controller.setP(IntakeConstants.kP);
        controller.setD(IntakeConstants.kD);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // The encoder is being dumb, so if it gives a really uneccesarily large reading it will try
        // zeroing it again.
        if(getCurrentPosition() > 100.0) {
            intakeEnc.setPosition(offsetInches);
        }

        if(targetInches > IntakeConstants.maxInches) targetInches = IntakeConstants.maxInches;
        else if(targetInches < IntakeConstants.minInches) targetInches = 0.0;

        if(!actuationIsManual) {
            controller.setReference(targetInches + offsetInches, ControlType.kPosition);
        }

        SmartDashboard.putNumber("intake inches", getCurrentPosition());
        SmartDashboard.putNumber("intake target", targetInches);
        
        SmartDashboard.putNumber("roller counts", upperMtr.getEncoder().getPosition());
    }

    // Put methods for controlling this subsystem here. Call these from Commands.

    public double getCurrentPosition() {
        return intakeEnc.getPosition() - offsetInches;
    }

    public void setActuationPower(double power) {
        if(
            (getCurrentPosition() <= IntakeConstants.minInches + IntakeConstants.manualControlPadding && power < 0.0) ||
            (getCurrentPosition() >= IntakeConstants.maxInches - IntakeConstants.manualControlPadding && power > 0.0)
        ) {
            actuationMtr.set(0.0);
        }
        else {
            actuationMtr.set(power);
        }
    }

    public void setRollerPower(double power) {
        upperMtr.set(power);
    }

    public void manualRollerControl(double power) {
        if(power != 0.0) {
            rollersAreManual = true;
            setRollerPower(power);
        }
        else {
            rollersAreManual = false;
        }
    }

    public void setRPM(double rpm) {
        if(!rollersAreManual) {
            double power = rpm / IntakeConstants.freeRPM;
            setRollerPower(power);
        }
    }

    public void setRelativeSpeed(double metersPerSecond) {
        if(!rollersAreManual) {
            double rpm = metersPerSecond * IntakeConstants.driveMetersPerSecondToRollerRPM;
            setRPM(rpm);
        }
    }

    public void setTarget(double inches) {
        targetInches = inches;
    }

    public void manualActuationControl(double manual) {
        SmartDashboard.putNumber("manual", manual);
        if(manual != 0) {
            actuationIsManual = true;
            setActuationPower(manual * IntakeConstants.manualPower);
        }
        else {
            if(actuationIsManual)
                setTarget(getCurrentPosition());

            actuationIsManual = false;
        }
    }
}