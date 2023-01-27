package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class DriveSys extends SubsystemBase {

    /**
     * Subsystem that controls the drivetrain of the robot
     * Handles all the odometry and base movement for the chassis
     */


    /**
     * SwerveModule objects
     * Parameters:
     * drive motor can ID
     * rotation motor can ID
     * external CANCoder can ID
     * measured CANCoder offset
     */

    private final SwerveModule frontLeft = 
        new SwerveModule(
            CANDevices.frontLeftDriveMotorId,
            CANDevices.frontLeftRotationMotorId,
            CANDevices.frontLeftRotationEncoderId,
            DriveConstants.frontLeftOffset
        );

    private final SwerveModule frontRight = 
        new SwerveModule(
            CANDevices.frontRightDriveMotorId,
            CANDevices.frontRightRotationMotorId,
            CANDevices.frontRightRotationEncoderId,
            DriveConstants.frontRightOffset
        );

    private final SwerveModule rearLeft = 
        new SwerveModule(
            CANDevices.rearLeftDriveMotorId,
            CANDevices.rearLeftRotationMotorId,
            CANDevices.rearLeftRotationEncoderId,
            DriveConstants.rearLeftOffset
        );

    private final SwerveModule rearRight = 
        new SwerveModule(
            CANDevices.rearRightDriveMotorId,
            CANDevices.rearRightRotationMotorId,
            CANDevices.rearRightRotationEncoderId,
            DriveConstants.rearRightOffset
        );

    PowerDistribution pdh; //TODO: Make its own subsystem?

    // commanded values from the joysticks and field relative value to use in AlignWithTargetVision and AlignWithGyro
    private double commandedForward = 0;
    private double commandedStrafe = 0;
    private double commandedRotation = 0;

    private boolean isCommandedLocked = false;

    private boolean isCommandedFieldRelative = false;

    private final PigeonIMU imu = new PigeonIMU(CANDevices.imuId);

    /**
     * odometry for the robot, measured in meters for linear motion and radians for rotational motion
     * Takes in kinematics and robot angle for parameters
     */
    // private final SwerveDriveOdometry odometry = 
    //     new SwerveDriveOdometry(
    //         DriveConstants.kinematics, 
    //         new Rotation2d(getHeading().getRadians()),
    //         new SwerveModulePosition[] {
    //             frontLeft.getPosition(),
    //             frontRight.getPosition(),
    //             rearLeft.getPosition(),
    //             rearRight.getPosition()
    //           }
    //     );

    // private final SwerveDriveOdometry odometry = 
    //     new SwerveDriveOdometry(
    //         DriveConstants.kinematics,
    //         getHeading(),
    //         getModulePositions(),
    //         new Pose2d()
    //     );

    private final SwerveDrivePoseEstimator odometry = 
        new SwerveDrivePoseEstimator(
            DriveConstants.kinematics, 
            getHeading(), 
            getModulePositions(), 
            new Pose2d()
        );

    public DriveSys() {

        imu.setYaw(0.0);

        // reset the measured distance driven for each module
        frontLeft.resetDistance();
        frontRight.resetDistance();
        rearLeft.resetDistance();
        rearRight.resetDistance();

        resetPose(new Pose2d(0, 0, new Rotation2d(0)));

        pdh = new PowerDistribution();
    }

    @Override
    public void periodic() {

        // update the odometry every 20ms
        odometry.update(getHeading(), getModulePositions());

        SmartDashboard.putNumber("heading", getHeading().getDegrees());
        SmartDashboard.putNumber("pitch", getPitch()); //TODO: Make sure pitch and roll values work
        SmartDashboard.putNumber("roll", getRoll());

        SmartDashboard.putNumber("Odometry x", odometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Odometry y", odometry.getEstimatedPosition().getY());

        SmartDashboard.putNumber("speed", getAverageDriveVelocityMetersPerSecond()); //TODO: Make sure this value seems accurate

        SmartDashboard.putNumber("front left rotation encoder", frontLeft.getRotationEncoderAngle().getDegrees());
        SmartDashboard.putNumber("front right rotation encoder", frontRight.getRotationEncoderAngle().getDegrees());
        SmartDashboard.putNumber("rear left rotation encoder", rearLeft.getRotationEncoderAngle().getDegrees());
        SmartDashboard.putNumber("rear right rotation encoder", rearRight.getRotationEncoderAngle().getDegrees());

        SmartDashboard.putNumber("front left CANcoder", frontLeft.getCanCoderAngle().getDegrees());
        SmartDashboard.putNumber("front right CANcoder", frontRight.getCanCoderAngle().getDegrees());
        SmartDashboard.putNumber("rear left CANcoder", rearLeft.getCanCoderAngle().getDegrees());
        SmartDashboard.putNumber("rear right CANcoder", rearRight.getCanCoderAngle().getDegrees());

        SmartDashboard.putNumber("front left distance", frontLeft.getDriveDistanceMeters());
        SmartDashboard.putNumber("front right distance", frontRight.getDriveDistanceMeters());
        SmartDashboard.putNumber("rear left distance", rearLeft.getDriveDistanceMeters());
        SmartDashboard.putNumber("rear right distance", rearRight.getDriveDistanceMeters());

        SmartDashboard.putNumber("current", pdh.getTotalCurrent());
    }
    
    /**
     * method for driving the robot
     * Parameters:
     * forward linear value
     * sideways linear value
     * rotation value
     * if the control is field relative or robot relative
     */
    public void drive(double forward, double strafe, double rotation, boolean isLocked, boolean isFieldRelative) {

        // update the drive inputs for use in AlignWithGyro and AlignWithTargetVision control
        commandedForward = forward;
        commandedStrafe = strafe;
        commandedRotation = rotation;

        isCommandedLocked = isLocked;

        isCommandedFieldRelative = isFieldRelative;

        SwerveModuleState[] states;
        if(isLocked) {
            // All wheels turn in 45 degrees to lock the drive base
            states = new SwerveModuleState[] {
                new SwerveModuleState(0.0, new Rotation2d(0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(-0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(-0.25 * Math.PI)),
                new SwerveModuleState(0.0, new Rotation2d(0.25 * Math.PI))
            };
        }
        else {
            // Represents the overall state of the drive base
            ChassisSpeeds speeds =
            isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    forward, strafe, rotation, getHeading())
                : new ChassisSpeeds(forward, strafe, rotation);

            // Uses kinematics (wheel placements) to convert overall robot state to array of individual module states
            states = DriveConstants.kinematics.toSwerveModuleStates(speeds);
        }

        // Makes sure the wheels don't try to spin faster than the maximum speed possible
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxDriveSpeedMetersPerSec);

        setModuleStates(states);

    }

    /**
     * Method to set the desired state for each swerve module
     * Uses PID and feedforward control to control the linear and rotational values for the modules
     */
    public void setModuleStates(SwerveModuleState[] moduleStates) {

        frontLeft.setDesiredState(moduleStates[0], false);
        frontRight.setDesiredState(moduleStates[1], false);
        rearLeft.setDesiredState(moduleStates[2], false);
        rearRight.setDesiredState(moduleStates[3], false);

    }

    public void setModuleStatesAuto(SwerveModuleState[] moduleStates) {
        frontLeft.setDesiredState(moduleStates[0], true);
        frontRight.setDesiredState(moduleStates[1], true);
        rearLeft.setDesiredState(moduleStates[2], true);
        rearRight.setDesiredState(moduleStates[3], true);
    }
    

    /**
     * @return An array of SwerveModuleState
     */
    public SwerveModuleState[] getModuleStates() {

        return new SwerveModuleState[] {
            new SwerveModuleState(frontLeft.getCurrentVelocityMetersPerSecond(), frontLeft.getRotationEncoderAngle()),
            new SwerveModuleState(frontRight.getCurrentVelocityMetersPerSecond(), frontRight.getRotationEncoderAngle()),
            new SwerveModuleState(rearLeft.getCurrentVelocityMetersPerSecond(), rearLeft.getRotationEncoderAngle()),
            new SwerveModuleState(rearRight.getCurrentVelocityMetersPerSecond(), rearRight.getRotationEncoderAngle())
        };

    }

    /**
     * @return An array of SwerveModulePosition
     */
    public SwerveModulePosition[] getModulePositions() {

        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        };

    }

    /**
     * @return The current estimated position of the robot on the field
     * based on drive encoder and gyro readings
     */
    public Pose2d getPose() {

        return odometry.getEstimatedPosition();

    }

    /*
     * Resets the current pose to a desired pose
     */
    public void resetPose(Pose2d pose) {

        imu.setYaw(0);
        odometry.resetPosition(getHeading(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition()
            }, 
        pose);

    }

    /*
     * Resets the measured distance driven for each module
     */
    public void resetDriveDistances() {

        frontLeft.resetDistance();
        frontRight.resetDistance();
        rearLeft.resetDistance();
        rearRight.resetDistance();

    }

    // return the average distance driven for each module to get an overall distance driven by the robot
    public double getAverageDriveDistanceMeters() {

        return ((
            frontLeft.getDriveDistanceMeters()
            + frontRight.getDriveDistanceMeters()
            + rearLeft.getDriveDistanceMeters()
            + rearRight.getDriveDistanceMeters())
            / 4.0);

    }

    // return the average velocity for each module to get an overall velocity for the robot
    public double getAverageDriveVelocityMetersPerSecond() {

        return ((
            Math.abs(frontLeft.getCurrentVelocityMetersPerSecond())
            + Math.abs(frontRight.getCurrentVelocityMetersPerSecond())
            + Math.abs(rearLeft.getCurrentVelocityMetersPerSecond() )
            + Math.abs(rearRight.getCurrentVelocityMetersPerSecond()))
            / 4.0);

    }

    // get the current heading of the robot based on the gyro
    public Rotation2d getHeading() {

        return Rotation2d.fromDegrees(imu.getYaw());

    }

    public double getPitch() {

        return imu.getPitch();

    }

    public double getRoll() {

        return imu.getRoll();

    }

    /**
     * Sets the gyro heading to zero.
     */
    public void zeroGyro() {
        imu.setYaw(0.0);
    }

    public double[] getCommandedDriveValues() {

        double[] values = {commandedForward, commandedStrafe, commandedRotation};

        return values;

    }

    public boolean getIsLocked() {

        return isCommandedLocked;

    }

    public boolean getIsFieldRelative() {

        return isCommandedFieldRelative;

    }

    public void resetImu() {

        imu.setYaw(0);

    }

}