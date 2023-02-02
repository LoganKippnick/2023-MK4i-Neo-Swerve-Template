package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * Class to represent and handle a swerve module
 * A module's state is measured by a CANCoder for the absolute position, integrated CANEncoder for relative position
 * for both steeration and linear movement
 */
public class SwerveModule extends SubsystemBase {

    private final CANSparkMax driveMtr;
    private final CANSparkMax steerMtr;

    private final RelativeEncoder driveEnc;
    private final RelativeEncoder steerEnc;

    private final CANCoder canCoder;

    // absolute offset for the CANCoder so that the wheels can be aligned when the robot is turned on
    private final Rotation2d offset;

    private final SparkMaxPIDController steerController;
    private final SparkMaxPIDController driveController;

    /**
     * Constructs a new SwerveModule.
     * 
     * <p>SwerveModule represents and handles a swerve module.
     * 
     * @param driveMtrId CAN ID of the NEO drive motor.
     * @param steerMtrId CAN ID of the NEO steeration motor.
     * @param canCoderId CAN ID of the CANCoder.
     * @param measuredOffsetRadians Offset of CANCoder reading from forward.
     */
    public SwerveModule(int driveMtrId, int steerMtrId, int canCoderId, double measuredOffsetRadians) {

        driveMtr = new CANSparkMax(driveMtrId, MotorType.kBrushless);
        steerMtr = new CANSparkMax(steerMtrId, MotorType.kBrushless);

        driveEnc = driveMtr.getEncoder();
        steerEnc = steerMtr.getEncoder();

        canCoder = new CANCoder(canCoderId);

        offset = new Rotation2d(measuredOffsetRadians);

        driveMtr.setIdleMode(IdleMode.kBrake);
        steerMtr.setIdleMode(IdleMode.kCoast);

        steerController = steerMtr.getPIDController();
        driveController = driveMtr.getPIDController();

        steerController.setP(DriveConstants.steerkP);
        steerController.setD(DriveConstants.steerkD);

        driveController.setP(DriveConstants.drivekP);

  

        //set the output of the drive encoder to be in radians for linear measurement
        driveEnc.setPositionConversionFactor(DriveConstants.driveMetersPerEncRev);

        //set the output of the drive encoder to be in radians per second for velocity measurement
        driveEnc.setVelocityConversionFactor(DriveConstants.driveMetersPerSecPerRPM);

        //set the output of the steeration encoder to be in radians
        steerEnc.setPositionConversionFactor(DriveConstants.steerRadiansPerEncRev);

        //configure the CANCoder to output in unsigned (wrap around from 360 to 0 degrees)
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        initsteerOffset();
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEnc.getPosition(), getCanCoderAngle());
    }

    /**
     * Resets the distance traveled by the module.
     */
    public void resetDistance() {

        driveEnc.setPosition(0.0);

    }

    /**
     * Returns the current drive distance of the module.
     * 
     * @return The current drive distance of the module.
     */
    public double getDriveDistanceMeters() {

        return driveEnc.getPosition();

    }
    
    /**
     * Returns the current angle of the module between 0 and 2 * PI.
     * 
     * @return The current angle of the module between 0 and 2 * PI.
     */
    public Rotation2d getCanCoderAngle() {

        double unsignedAngle = (Units.degreesToRadians(canCoder.getAbsolutePosition()) - offset.getRadians()) % (2 * Math.PI);

        return new Rotation2d(unsignedAngle);

    }

    /**
     * Returns the current absolute angle of the module from the steeration motor encoder.
     * 
     * @return The curretn absolute angle of the module.
     */
    public Rotation2d getSteerEncAngle() {

        return new Rotation2d(steerEnc.getPosition());

    }

    /**
     * Returns the current velocity of the module from the drive motor encoder.
     * 
     * @return The current velocity of the module in meters per second.
     */
    public double getCurrentVelocityMetersPerSecond() {

        return driveEnc.getVelocity();
        
    }

    /**
     * Calculates the angle motor setpoint based on the desired angle and the current angle measurement.
     */
    // TODO: Figure out how necessary this method is.
    public double calculateAdjustedAngle(double targetAngle, double currentAngle) {

        double modAngle = currentAngle % (2.0 * Math.PI);

        if (modAngle < 0.0) modAngle += 2.0 * Math.PI;
        
        double newTarget = targetAngle + currentAngle - modAngle;

        if (targetAngle - modAngle > Math.PI) newTarget -= 2.0 * Math.PI;
        else if (targetAngle - modAngle < -Math.PI) newTarget += 2.0 * Math.PI;

        return newTarget;

    }

    /**
     * Initializes the steeration motor encoder to the value of the CANCoder, accounting for the offset.
     */
    public void initsteerOffset() {

        steerEnc.setPosition(getCanCoderAngle().getRadians());

    }

    /**
     * Sets the desired state of the swerve module and optimizes it.
     * <p>If closed-loop, uses PID and a feedforward to control the speed.
     * If open-loop, sets the speed to a percentage. Open-loop control should
     * only be used if running an autonomour trajectory.
     *
     * @param desiredState Object that holds a desired linear and steerational setpoint.
     * @param isOpenLoop True if the velocity control is open- or closed-loop.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        
        // Optimizes speed and angle to minimize change in heading
        // (e.g. module turns 1 degree and reverses drive direction to get from 90 degrees to -89 degrees)
        desiredState = SwerveModuleState.optimize(desiredState, getSteerEncAngle());

        steerController.setReference(
            calculateAdjustedAngle(
                desiredState.angle.getRadians(),
                getSteerEncAngle().getRadians()),
            ControlType.kPosition
        );

        if(isOpenLoop) {
            driveMtr.set(desiredState.speedMetersPerSecond / DriveConstants.kFreeMetersPerSecond);
        }
        else {
            double speedMetersPerSecond = desiredState.speedMetersPerSecond * DriveConstants.maxDriveSpeedMetersPerSec;

            driveController.setReference(
                speedMetersPerSecond,
                ControlType.kVelocity,
                0, 
                DriveConstants.driveFF.calculate(speedMetersPerSecond)
            );
        }
    }    
}