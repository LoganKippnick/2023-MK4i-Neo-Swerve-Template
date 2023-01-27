package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class CANDevices {

        public static final int frontLeftRotationEncoderId = 1;
        public static final int frontLeftRotationMotorId = 9;
        public static final int frontLeftDriveMotorId = 5;

        public static final int frontRightRotationEncoderId = 2;
        public static final int frontRightRotationMotorId = 10;
        public static final int frontRightDriveMotorId = 6;

        public static final int rearLeftRotationEncoderId = 3;
        public static final int rearLeftRotationMotorId = 11;
        public static final int rearLeftDriveMotorId = 7;

        public static final int rearRightRotationEncoderId = 4;
        public static final int rearRightRotationMotorId = 12;
        public static final int rearRightDriveMotorId = 8;

        public static final int imuId = 13;

    }

    public static final class Controllers {

        public static final int leftJoystickPort = 0;
        public static final int rightJoystickPort = 1;

        public static final int operatorControllerPort = 2;

        public static final double operatorControllerDeadband = 0.1;

    }

    public static final class PneumaticChannels {

        public  static final int PCMId = 19;

        public static final int[] intakeSolenoidChannels = {0, 1};
        public static final int lockingSolenoidChannel = 2;


    }

    public static final class DriveConstants {

        public static final double trackWidth = Units.inchesToMeters(29.5);
        public static final double wheelBase = Units.inchesToMeters(29.5);


        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0), //front left
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), //front right
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), //rear left
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) //rear right
            );

        public static final double driveWheelGearReduction = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));
        public static final double rotationWheelGearReduction = 1 / ((14.0 / 50.0) * (10.0 / 60.0));

        public static final double wheelDiameterMeters = 0.050686 * 2;

        public static final double driveMetersPerEncoderRev = (wheelDiameterMeters * Math.PI) / driveWheelGearReduction;
        public static final double driveEncoderRPMperMPS = driveMetersPerEncoderRev / 60;

        public static final double kFreeMetersPerSecond = 5600 * driveEncoderRPMperMPS;

        public static final double rotationMotorMaxSpeedRadPerSec = 2.0;
        public static final double rotationMotorMaxAccelRadPerSecSq = 1.0;

        public static final double maxDriveSpeedMetersPerSec = 3; //3
        public static final double maxDriveAccelMetersPerSecSq = 5;

        public static final double maxTurnRateRadiansPerSec = 2 * Math.PI; //Rate the robot will spin with full rotation command
        public static final double maxTurnAccelRadiansPerSecSq = Math.PI;

        public static final double frontLeftOffset = Units.degreesToRadians(105.420);
        public static final double frontRightOffset = Units.degreesToRadians(228.428);
        public static final double rearLeftOffset = Units.degreesToRadians(292.139);
        public static final double rearRightOffset = Units.degreesToRadians(313.183);

        public static final double drivekP = 0.005;

        public static final double steerkP = 1.6636 * 0.75; //0.3
        public static final double steerkD = 1.2083 * 0.75; //0.2

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.667, 2.44, 0); //TODO: Input FF constants below into FF to see if it still functions

        public static final double ksVolts = .055;
        public static final double kvVoltSecondsPerMeter = .2;
        public static final double kaVoltSecondsSquaredPerMeter = .02;
    }

    public static final class AutoConstants {

        public static final double maxVelMetersPerSec = 2;
        public static final double maxAccelMetersPerSecondSq = 1;

        public static final double drivekP = 2.25;
        public static final double drivekD = 0.0;

        public static final double rotationkP = 1.5;
        public static final double rotationkD = 0.25;

        public final static TrajectoryConfig config = 
            new TrajectoryConfig(
                AutoConstants.maxVelMetersPerSec, 
                AutoConstants.maxAccelMetersPerSecondSq
            )
            .setKinematics(DriveConstants.kinematics
        );

        public static final PIDController driveController = new PIDController(
            drivekP,
            0,
            drivekD
        );

        public static final ProfiledPIDController rotationController = constructRotationController();
        private static ProfiledPIDController constructRotationController() {
            ProfiledPIDController rotationController = new ProfiledPIDController(
                rotationkP,
                0,
                rotationkD,
                new TrapezoidProfile.Constraints(
                    DriveConstants.maxTurnRateRadiansPerSec,
                    DriveConstants.maxTurnRateRadiansPerSec
                )
            );

            // sets wraparound from 0 to 2 * PI
            rotationController.enableContinuousInput(0, 2 * Math.PI);
            
            return rotationController;
        }
    }    
}
