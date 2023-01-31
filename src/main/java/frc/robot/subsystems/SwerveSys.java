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

public class SwerveSys extends SubsystemBase {

    // Initializes swerve module objects
    private final SwerveModule frontLeftMod = 
        new SwerveModule(
            CANDevices.frontLeftDriveMtrId,
            CANDevices.frontLeftRotMtrId,
            CANDevices.frontLeftRotEncId,
            DriveConstants.frontLeftModOffset
        );

    private final SwerveModule frontRightMod = 
        new SwerveModule(
            CANDevices.frontRightDriveMtrId,
            CANDevices.frontRightRotMtrId,
            CANDevices.frontRightRotEncId,
            DriveConstants.frontRightModOffset
        );

    private final SwerveModule rearLeftMod = 
        new SwerveModule(
            CANDevices.rearLeftDriveMtrId,
            CANDevices.rearLeftRotMtrId,
            CANDevices.rearLeftRotEncId,
            DriveConstants.rearLeftModOffset
        );

    private final SwerveModule rearRightMod = 
        new SwerveModule(
            CANDevices.rearRightDriveMtrId,
            CANDevices.rearRightRotMtrId,
            CANDevices.rearRightRotEncId,
            DriveConstants.rearRightModOffset
        );

    PowerDistribution pdh; // TODO: Make its own subsystem? For use with compressor.

    // Commanded values from the joysticks and field relative value for use in driving assistance commands
    private double commandedDriveX = 0;
    private double commandedDriveY = 0;
    private double commandedRotation = 0;

    private boolean isCommandedLocked = false;

    private boolean isCommandedFieldOriented = false;

    private final PigeonIMU imu = new PigeonIMU(CANDevices.imuId);

    // Odometry for the robot, measured in meters for linear motion and radians for rotational motion
    // Takes in kinematics and robot angle for parameters

    private SwerveDrivePoseEstimator odometry = 
        new SwerveDrivePoseEstimator(
            DriveConstants.kinematics, 
            getHeading(), 
            getModulePositions(), 
            new Pose2d()
        );

    /**
     * Constructs a new SwerveSys.
     * 
     * <p>SwerveCmd contains 4 {@link SwerveModule}, a gyro, and methods to control the drive base and odometry.
     */
    public SwerveSys() {

        // Resets the measured distance driven for each module
        frontLeftMod.resetDistance();
        frontRightMod.resetDistance();
        rearLeftMod.resetDistance();
        rearRightMod.resetDistance();

        resetPose();

        pdh = new PowerDistribution();
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {

        // Updates the odometry every 20ms
        odometry.update(getHeading(), getModulePositions());

        SmartDashboard.putNumber("heading", getHeading().getDegrees());
        SmartDashboard.putNumber("pitch", getPitch()); // TODO: Make sure pitch and roll values work (and figure out which one we need)
        SmartDashboard.putNumber("roll", getRoll()); // TODO: Make methods for resetting pitch and/or roll

        SmartDashboard.putNumber("Odometry x", odometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Odometry y", odometry.getEstimatedPosition().getY());

        SmartDashboard.putNumber("speed", getAverageDriveVelocityMetersPerSecond());

        SmartDashboard.putNumber("front left rotation encoder", frontLeftMod.getRotEncAngle().getDegrees());
        SmartDashboard.putNumber("front right rotation encoder", frontRightMod.getRotEncAngle().getDegrees());
        SmartDashboard.putNumber("rear left rotation encoder", rearLeftMod.getRotEncAngle().getDegrees());
        SmartDashboard.putNumber("rear right rotation encoder", rearRightMod.getRotEncAngle().getDegrees());

        SmartDashboard.putNumber("front left CANcoder", frontLeftMod.getCanCoderAngle().getDegrees());
        SmartDashboard.putNumber("front right CANcoder", frontRightMod.getCanCoderAngle().getDegrees());
        SmartDashboard.putNumber("rear left CANcoder", rearLeftMod.getCanCoderAngle().getDegrees());
        SmartDashboard.putNumber("rear right CANcoder", rearRightMod.getCanCoderAngle().getDegrees());

        SmartDashboard.putNumber("front left distance", frontLeftMod.getDriveDistanceMeters());
        SmartDashboard.putNumber("front right distance", frontRightMod.getDriveDistanceMeters());
        SmartDashboard.putNumber("rear left distance", rearLeftMod.getDriveDistanceMeters());
        SmartDashboard.putNumber("rear right distance", rearRightMod.getDriveDistanceMeters());

        SmartDashboard.putNumber("current", pdh.getTotalCurrent());
    }
    
    /**
     * Inputs drive values into subsystem.
     * 
     * @param driveX forward/backward lateral motion.
     * @param driveY left/right lateral motion.
     * @param rotation rotational motion.
     * @param isLocked overrides driving and turns all modules in at a 45-degree angle to lock modules.
     * @param isFieldOriented whether driving is field- or robot-oriented.
     */
    public void drive(double driveX, double driveY, double rotation, boolean isLocked, boolean isFieldOriented) {

        // Updates the drive inputs for use in AlignWithGyro and AlignWithTargetVision control
        commandedDriveX = driveX;
        commandedDriveY = driveY;
        commandedRotation = rotation;

        isCommandedLocked = isLocked;

        isCommandedFieldOriented = isFieldOriented;

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
            // Represents the overall state of the drive base.
            ChassisSpeeds speeds =
            isFieldOriented
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveX, driveY, rotation, getHeading())
                : new ChassisSpeeds(driveX, driveY, rotation);

            // Uses kinematics (wheel placements) to convert overall robot state to array of individual module states
            states = DriveConstants.kinematics.toSwerveModuleStates(speeds);
        }

        // Makes sure the wheels don't try to spin faster than the maximum speed possible
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxDriveSpeedMetersPerSec);

        setModuleStates(states);

    }

    /**
     * Sets the desired state for each swerve module.
     * <p>Uses PID and feedforward control (closed-loop) to control the linear and rotational values for the modules.
     * 
     * @param moduleStates the module states to set.
     */
    public void setModuleStates(SwerveModuleState[] moduleStates) {

        frontLeftMod.setDesiredState(moduleStates[0], false);
        frontRightMod.setDesiredState(moduleStates[1], false);
        rearLeftMod.setDesiredState(moduleStates[2], false);
        rearRightMod.setDesiredState(moduleStates[3], false);

    }

    /**
     * Sets the desired state for each swerve module.
     * <p>Uses PID and feedforward control to control the linear and rotational values for the modules.
     * 
     * @param moduleStates the module states to set.
     */
    public void setModuleStatesAuto(SwerveModuleState[] moduleStates) {
        frontLeftMod.setDesiredState(moduleStates[0], true);
        frontRightMod.setDesiredState(moduleStates[1], true);
        rearLeftMod.setDesiredState(moduleStates[2], true);
        rearRightMod.setDesiredState(moduleStates[3], true);
    }
    

    /**
     * Returns an array of module states.
     * 
     * @return An array of SwerveModuleState.
     */
    public SwerveModuleState[] getModuleStates() {

        return new SwerveModuleState[] {
            new SwerveModuleState(frontLeftMod.getCurrentVelocityMetersPerSecond(), frontLeftMod.getRotEncAngle()),
            new SwerveModuleState(frontRightMod.getCurrentVelocityMetersPerSecond(), frontRightMod.getRotEncAngle()),
            new SwerveModuleState(rearLeftMod.getCurrentVelocityMetersPerSecond(), rearLeftMod.getRotEncAngle()),
            new SwerveModuleState(rearRightMod.getCurrentVelocityMetersPerSecond(), rearRightMod.getRotEncAngle())
        };

    }

    /**
     * Returns an array of module positions.
     * 
     * @return An array of SwerveModulePosition.
     */
    public SwerveModulePosition[] getModulePositions() {

        return new SwerveModulePosition[] {
            frontLeftMod.getPosition(),
            frontRightMod.getPosition(),
            rearLeftMod.getPosition(),
            rearRightMod.getPosition()
        };

    }

    /**
     * @return The current estimated position of the robot on the field
     * based on drive encoder and gyro readings.
     */
    public Pose2d getPose() {

        return odometry.getEstimatedPosition();

    }

    /**
     * Resets the current pose.
     */
    // TODO: Make sure this works.
    public void resetPose() {

        resetHeading();
        resetDriveDistances();

    }

    /**
     * Sets the pose of the robot.
     * 
     * @param pose The pose to set the robot to.
     */
    // TODO: Make sure this works.
    public void setPose(Pose2d pose) {

        imu.setYaw(pose.getRotation().getDegrees());

        odometry = new SwerveDrivePoseEstimator(
            DriveConstants.kinematics,
            pose.getRotation(),
            getModulePositions(),
            pose
        );

    }

    /**
     * Resets the measured distance driven for each module.
     */
    public void resetDriveDistances() {

        frontLeftMod.resetDistance();
        frontRightMod.resetDistance();
        rearLeftMod.resetDistance();
        rearRightMod.resetDistance();

    }

    /**
     * Returns the average distance driven for each module to get an overall distance driven by the robot.
     * 
     * @return The overall distance driven by the robot in meters.
     */
    public double getAverageDriveDistanceMeters() {

        return ((
            frontLeftMod.getDriveDistanceMeters()
            + frontRightMod.getDriveDistanceMeters()
            + rearLeftMod.getDriveDistanceMeters()
            + rearRightMod.getDriveDistanceMeters())
            / 4.0);

    }

    // 
    /**
     * Returns the average velocity for each module to get an overall velocity for the robot.
     * 
     * @return The overall velocity of the robot in meters per second.
     */
    public double getAverageDriveVelocityMetersPerSecond() {

        return ((
            Math.abs(frontLeftMod.getCurrentVelocityMetersPerSecond())
            + Math.abs(frontRightMod.getCurrentVelocityMetersPerSecond())
            + Math.abs(rearLeftMod.getCurrentVelocityMetersPerSecond() )
            + Math.abs(rearRightMod.getCurrentVelocityMetersPerSecond()))
            / 4.0);

    }

    /**
     * Returns the current heading of the robot from the gyro.
     * 
     * @return The current heading of the robot.
     */
    public Rotation2d getHeading() {

        return Rotation2d.fromDegrees(imu.getYaw());

    }

    /**
     * Returns the current pitch of the robot from the gyro.
     * 
     * @return The current pitch of the robot.
     */
    public double getPitch() {

        return imu.getPitch();

    }

    /**
     * Returns the current roll of the robot from the gyro.
     * 
     * @return The current roll of the robot.
     */
    public double getRoll() {

        return imu.getRoll();

    }

    /**
     * Sets the gyro heading to zero.
     */
    public void resetHeading() {

        imu.setYaw(0.0);

    }

    /**
     * Returns an array of the commanded drive values.
     * 
     * @return An array of the commanded drive values.
     */
    public double[] getCommandedDriveValues() {

        return new double[] {commandedDriveX, commandedDriveY, commandedRotation};

    }

    /**
     * Returns whether the wheels are "locked" in at a 45 degree angles
     * 
     * @return True if the wheels are locked
     */
    public boolean getIsLocked() {

        return isCommandedLocked;

    }

    /**
     * Returns whether the driver controls are field-oriented. If false, the controls are robot-oriented.
     * 
     * @return True if the driver controls are field-oriented.
     */
    public boolean getIsFieldOriented() {

        return isCommandedFieldOriented;

    }
}