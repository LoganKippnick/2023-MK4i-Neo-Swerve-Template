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

public class SwerveModule extends SubsystemBase {

    /**
     * Class to represent and handle a swerve module
     * A module's state is measured by a CANCoder for the absolute position, integrated CANEncoder for relative position
     * for both rotation and linear movement
     */


    private final CANSparkMax driveMotor;
    private final CANSparkMax rotationMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotationEncoder;

    private final CANCoder canCoder;

    //absolute offset for the CANCoder so that the wheels can be aligned when the robot is turned on
    private final Rotation2d offset;

    private final SparkMaxPIDController rotationController;
    private final SparkMaxPIDController driveController;

    public SwerveModule(
        int driveMotorId, 
        int rotationMotorId,
        int canCoderId,
        double measuredOffsetRadians
    ) {

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationMotorId, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        rotationEncoder = rotationMotor.getEncoder();

        canCoder = new CANCoder(canCoderId);

        offset = new Rotation2d(measuredOffsetRadians);

        driveMotor.setIdleMode(IdleMode.kBrake);
        rotationMotor.setIdleMode(IdleMode.kCoast);

        rotationController = rotationMotor.getPIDController();
        driveController = driveMotor.getPIDController();

        rotationController.setP(DriveConstants.rotationkP);
        rotationController.setD(DriveConstants.rotationkD);

        driveController.setP(DriveConstants.drivekP);

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

    public void resetDistance() {

        driveEncoder.setPosition(0.0);

    }

    public double getDriveDistanceMeters() {

        return driveEncoder.getPosition();

    }
    
    public Rotation2d getCanCoderAngle() {

        double unsignedAngle = (Units.degreesToRadians(canCoder.getAbsolutePosition()) - offset.getRadians()) % (2 * Math.PI);

        return new Rotation2d(unsignedAngle);

    }

    public Rotation2d getRotationEncoderAngle() {

        double unsignedAngle = rotationEncoder.getPosition();

        return new Rotation2d(unsignedAngle);

    }

    public double getCurrentVelocityMetersPerSecond() {

        return driveEncoder.getVelocity();
        
    }

    // public double getCurrentVelocityMetersPerSecond() {

    //     return driveEncoder.getVelocity() * (DriveConstants.wheelDiameterMeters / 2.0);

    // }
    // Revert m/s to this if changing velocityConversionFactor doesn't work

    //calculate the angle motor setpoint based on the desired angle and the current angle measurement
    public double calculateAdjustedAngle(double targetAngle, double currentAngle) {

        double modAngle = currentAngle % (2.0 * Math.PI);

        if (modAngle < 0.0) modAngle += 2.0 * Math.PI;
        
        double newTarget = targetAngle + currentAngle - modAngle;

        if (targetAngle - modAngle > Math.PI) newTarget -= 2.0 * Math.PI;
        else if (targetAngle - modAngle < -Math.PI) newTarget += 2.0 * Math.PI;

        return newTarget;

    }

    //initialize the integrated CANCoder to offset measurement by the CANCoder reading
    public void initRotationOffset() {

        rotationEncoder.setPosition(getCanCoderAngle().getRadians());

    }

    /**
     * Method to set the desired state of the swerve module
     * Parameter: 
     * SwerveModuleState object that holds a desired linear and rotational setpoint
     * Uses PID and a feedforward to control the output
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        
        rotationController.setReference(
            calculateAdjustedAngle(
                desiredState.angle.getRadians(),
                getRotationEncoderAngle().getRadians()),
            ControlType.kPosition
        );

        if(isOpenLoop) {
            driveMotor.set(desiredState.speedMetersPerSecond / DriveConstants.kFreeMetersPerSecond);
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

    public void resetEncoders() {

        driveEncoder.setPosition(0);
        rotationEncoder.setPosition(0);

    }
    
}