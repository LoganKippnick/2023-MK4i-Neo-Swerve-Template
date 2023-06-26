package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.IntakeConstants;

public class IntakeSys extends SubsystemBase {

    // Declare actuators, sensors, and other variables here

    private final CANSparkMax actuationMtr;
    private final CANSparkMax rollerMtr;

    private final RelativeEncoder intakeEnc;

    private final SparkMaxPIDController actuationController;
    private final SparkMaxPIDController rollerController;

    private double targetInches = 0.0;

    private boolean actuationIsManual = false;
    private boolean rollersAreManual = false;
    private boolean rollersAreRelative = false;

    private DoubleSupplier robotSpeedMetersPerSecond;
    private double relativeSpeed = 0.0;

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
    public IntakeSys(DoubleSupplier robotSpeedMetersPerSecond) {
        // Initialize and configure actuators and sensors here
        actuationMtr = new CANSparkMax(CANDevices.actuationMtrId, MotorType.kBrushed);
        actuationMtr.setInverted(true);
        actuationMtr.setSmartCurrentLimit(IntakeConstants.actuationCurrentLimitAmps);

        rollerMtr = new CANSparkMax(CANDevices.rollerMtrId, MotorType.kBrushless);
        rollerMtr.setInverted(true);
        rollerMtr.getEncoder().setVelocityConversionFactor(IntakeConstants.rollerGearReduction);
        rollerMtr.setIdleMode(IdleMode.kBrake);
        rollerMtr.setSmartCurrentLimit(IntakeConstants.rollerCurrentLimitAmps);

        intakeEnc = actuationMtr.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, IntakeConstants.actuationEncCountsPerRev);
        intakeEnc.setAverageDepth(64);
        intakeEnc.setPosition(offsetInches);
        intakeEnc.setInverted(false);
        intakeEnc.setPositionConversionFactor(IntakeConstants.encRevToInches);

        this.robotSpeedMetersPerSecond = robotSpeedMetersPerSecond;
        // relativeSpeed = IntakeConstants.rollerRelativeMetersPerSecond;

        actuationController = actuationMtr.getPIDController();
        actuationController.setP(IntakeConstants.kP);
        actuationController.setD(IntakeConstants.kD);

        rollerController = rollerMtr.getPIDController();
        rollerController.setFF(0.0002525);
        rollerController.setP(0.00015);
        rollerController.setI(0.000001);
        rollerController.setIZone(30.0);
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
            actuationController.setReference(targetInches + offsetInches, ControlType.kPosition);
        }

        if(!rollersAreManual) {
            if(!actuationIsManual && getCurrentPosition() > IntakeConstants.intakeRollerStartInches && targetInches == IntakeConstants.outInches) {
                if(rollersAreRelative)
                    setRPM((relativeSpeed - (robotSpeedMetersPerSecond.getAsDouble() * IntakeConstants.rollerRelativeSpeedFactor)) * IntakeConstants.driveMetersPerSecondToRollerRPM);
                else
                    setRPM(relativeSpeed * IntakeConstants.driveMetersPerSecondToRollerRPM);
            }
            else {
                setRPM(0.0);
            }
        }
        SmartDashboard.putNumber("roller power", rollerMtr.get());

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

    public void manualRollerControl(double power) {
        if(power != 0.0) {
            rollersAreManual = true;
            rollerMtr.set(power);
        }
        else {
            rollersAreManual = false;
        }
    }

    public void setRPM(double rpm) {
        // double power = rpm / IntakeConstants.freeRPM;
        // rollerMtr.set(power);
        rollerController.setReference(rpm, ControlType.kVelocity);
    }

    public double getCurrentSpeedRPM() {
        return rollerMtr.getEncoder().getVelocity();
    }

    public double getCurrentSpeedMetersPerSecond() {
        return getCurrentSpeedRPM() / IntakeConstants.driveMetersPerSecondToRollerRPM;
    }

    public void setRelativeSpeed(double metersPerSecond) {
        relativeSpeed = metersPerSecond;
        rollersAreRelative = true;
    }

    public void setAbsoluteSpeed(double metersPerSecond) {
        relativeSpeed = metersPerSecond;
        rollersAreRelative = false;
    }

    public void disableRelativeSpeed() {
        relativeSpeed = 0.0;
    }

    public double getTargetInches() {
        return targetInches;
    }

    public void setTarget(double inches) {
        targetInches = inches;
    }

    public boolean isAtTarget() {
        return getCurrentPosition() > targetInches - IntakeConstants.targetToleranceInches &&
            getCurrentPosition() < targetInches + IntakeConstants.targetToleranceInches;
    }

    public void manualActuationControl(double manual) {
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