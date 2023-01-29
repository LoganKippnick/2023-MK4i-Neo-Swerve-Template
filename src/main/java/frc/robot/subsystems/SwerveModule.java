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
 * for both rotation and linear movement
 */
public class SwerveModule extends SubsystemBase {

    private final CANSparkMax driveMtr;
    private final CANSparkMax rotationMtr;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotationEncoder;

    private final CANCoder canCoder;

    // absolute offset for the CANCoder so that the wheels can be aligned when the robot is turned on
    private final Rotation2d offset;

    private final SparkMaxPIDController rotationController;
    private final SparkMaxPIDController driveController;

    /**
     * Constructs a new SwerveModule.
     * 
     * <p>SwerveModule represents and handles a swerve module.
     * 
     * @param driveMtrId CAN ID of the NEO drive motor.
     * @param rotationMtrId CAN ID of the NEO rotation motor.
     * @param canCoderId CAN ID of the CANCoder.
     * @param measuredOffsetRadians Offset of CANCoder reading from forward.
     */
    public SwerveModule(int driveMtrId, int rotationMtrId, int canCoderId, double measuredOffsetRadians) {

        driveMtr = new CANSparkMax(driveMtrId, MotorType.kBrushless);
        rotationMtr = new CANSparkMax(rotationMtrId, MotorType.kBrushless);

        driveEncoder = driveMtr.getEncoder();
        rotationEncoder = rotationMtr.getEncoder();

        canCoder = new CANCoder(canCoderId);

        offset = new Rotation2d(measuredOffsetRadians);

        driveMtr.setIdleMode(IdleMode.kBrake);
        rotationMtr.setIdleMode(IdleMode.kCoast);

        rotationController = rotationMtr.getPIDController();
        driveController = driveMtr.getPIDController();

        rotationController.setP(DriveConstants.steerkP);
        rotationController.setD(DriveConstants.steerkD);

        driveController.setP(DriveConstants.drivekP);
        // TODO: Try this for acceleration control, if necessary, probably with a method in DriveSys. Could implement for driving with arm extended.
        // driveController.setSmartMotionMaxAccel(
        //     DriveConstants.maxDriveAccelMetersPerSecSq * Math.PI * DriveConstants.wheelDiameterMeters * 60,
        //     0
        // );
  

        //set the output of the drive encoder to be in radians for linear measurement
        driveEncoder.setPositionConversionFactor(DriveConstants.driveMetersPerEncoderRev);

        //set the output of the drive encoder to be in radians per second for velocity measurement
        driveEncoder.setVelocityConversionFactor(DriveConstants.driveEncoderRPMperMPS);

        //set the output of the rotation encoder to be in radians
        rotationEncoder.setPositionConversionFactor(2 * Math.PI / DriveConstants.rotationWheelGearReduction);

        //configure the CANCoder to output in unsigned (wrap around from 360 to 0 degrees)
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        initRotationOffset();
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), getCanCoderAngle());
    }

    /**
     * Resets the distance traveled by the module.
     */
    public void resetDistance() {

        driveEncoder.setPosition(0.0);

    }

    /**
     * Returns the current drive distance of the module.
     * 
     * @return The current drive distance of the module.
     */
    public double getDriveDistanceMeters() {

        return driveEncoder.getPosition();

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
     * Returns the current absolute angle of the module from the rotation motor encoder.
     * 
     * @return The curretn absolute angle of the module.
     */
    public Rotation2d getRotationEncoderAngle() {

        return new Rotation2d(rotationEncoder.getPosition());

    }

    /**
     * Returns the current velocity of the module from the drive motor encoder.
     * 
     * @return The current velocity of the module in meters per second.
     */
    public double getCurrentVelocityMetersPerSecond() {

        return driveEncoder.getVelocity();
        
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
     * Initializes the rotation motor encoder to the value of the CANCoder, accounting for the offset.
     */
    public void initRotationOffset() {

        rotationEncoder.setPosition(getCanCoderAngle().getRadians());

    }

    /**
     * Sets the desired state of the swerve module and optimizes it.
     * <p>If closed-loop, uses PID and a feedforward to control the speed.
     * If open-loop, sets the speed to a percentage. Open-loop control should
     * only be used if running an autonomour trajectory.
     *
     * @param desiredState Object that holds a desired linear and rotational setpoint.
     * @param isOpenLoop True if the velocity control is open- or closed-loop.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        
        // Optimizes speed and angle to minimize change in heading
        // (e.g. module turns 1 degree and reverses drive direction to get from 90 degrees to -89 degrees)
        desiredState = SwerveModuleState.optimize(desiredState, getRotationEncoderAngle());

        rotationController.setReference(
            calculateAdjustedAngle(
                desiredState.angle.getRadians(),
                getRotationEncoderAngle().getRadians()),
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