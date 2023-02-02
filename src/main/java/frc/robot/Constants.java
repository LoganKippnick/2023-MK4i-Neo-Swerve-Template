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

        public static final int frontLeftRotEncId = 1;
        public static final int frontLeftRotMtrId = 9;
        public static final int frontLeftDriveMtrId = 5;

        public static final int frontRightRotEncId = 2;
        public static final int frontRightRotMtrId = 10;
        public static final int frontRightDriveMtrId = 6;

        public static final int rearLeftRotEncId = 3;
        public static final int rearLeftRotMtrId = 11;
        public static final int rearLeftDriveMtrId = 7;

        public static final int rearRightRotEncId = 4;
        public static final int rearRightRotMtrId = 12;
        public static final int rearRightDriveMtrId = 8;

        public static final int imuId = 13;

        public static final int pneumaticHubId = 14;

        public static final int masterMtrId = 15;
        public static final int slaveMtrId = 16;
    }

    public static final class Controllers {

        public static final int driverControllerPort = 0;
        public static final int operatorControllerPort = 1;

        public static final double operatorControllerDeadband = 0.1;

    }

    public static final class PneumaticChannels {

        public  static final int PCMId = 16;

        public static final int[] liftSolsCh = {0, 1};

        public static final int[] clawSolsCh = {2, 3};

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

        public static final double driveMtrGearReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static final double rotMtrGearReduction = (14.0 / 50.0) * (10.0 / 60.0);

        public static final double wheelRadiusMeters = 0.050686;
        public static final double wheelCircumferenceMeters = 2 * wheelRadiusMeters * Math.PI;

        public static final double driveMetersPerEncRev = wheelCircumferenceMeters * driveMtrGearReduction;
        public static final double driveMetersPerSecPerRPM = driveMetersPerEncRev / 60;

        public static final double turnRadiansPerEncRev = 2 * Math.PI * DriveConstants.rotMtrGearReduction;

        public static final double kFreeMetersPerSecond = 5600 * driveMetersPerSecPerRPM;

        public static final double rotMtrMaxSpeedRadPerSec = 2.0;
        public static final double rotMtrMaxAccelRadPerSecSq = 1.0;

        public static final double maxDriveSpeedMetersPerSec = 3; //3

        public static final double maxTurnRateRadiansPerSec = 2 * Math.PI; //Rate the robot will spin with full Rot command

        public static final double frontLeftModOffset = Units.degreesToRadians(105.420);
        public static final double frontRightModOffset = Units.degreesToRadians(228.428);
        public static final double rearLeftModOffset = Units.degreesToRadians(292.139);
        public static final double rearRightModOffset = Units.degreesToRadians(313.183);

        public static final double drivekP = 0.005;

        public static final double steerkP = 1.2477;
        public static final double steerkD = 0.9062/*25*/;

        public static final double ksVolts = .055;
        public static final double kvVoltSecsPerMeter = .2;
        public static final double kaVoltSecsPerMeterSq = .02;

        // TODO: Input FF constants above into FF to see if it works better. If not, change constants to match: ks = 0.667; kv = 2.44; ka = 0
        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.667, 2.44, 0);
    }

    public static final class AutoConstants {

        public static final double maxVelMetersPerSec = 2;
        public static final double maxAccelMetersPerSecondSq = 1;

        // TODO: Tune the PID values below. Rotation should be okay.
        public static final double drivekP = 2.25;
        public static final double drivekD = 0.0;

        public static final double rotkP = 1.5;
        public static final double rotkD = 0.25;

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

        public static final ProfiledPIDController rotController = constructRotController();
        private static ProfiledPIDController constructRotController() {
            ProfiledPIDController rotController = new ProfiledPIDController(
                rotkP,
                0,
                rotkD,
                new TrapezoidProfile.Constraints(
                    DriveConstants.maxTurnRateRadiansPerSec,
                    DriveConstants.maxTurnRateRadiansPerSec
                )
            );

            // sets wraparound from 0 to 2 * PI
            rotController.enableContinuousInput(0, 2 * Math.PI);
            
            return rotController;
        }

        public static final String trajectoryFileLocation = "pathplanner/generatedJSON/";
    }
    
    public static final class LiftConstants {

        public static final double gearReduction = 1.0 / 5.0;

        public static final float maxHeightInches = (float) 74.0;
        public static final double inchesPerEncRev = 11 * gearReduction;

        public static final int smartMotionPIDSlot = 0;

        public static final double kP = 0.1;
        // TODO: Find out if kI is necessary (used if reading is slightly off from target).
        public static final double kI = 0.0001;
        public static final double kD = 2.0;
        public static final double kFF = 0.0001;
        
        public static final double kIZone = 0.25;

        public static final double feetPerSecondPerRPM = (inchesPerEncRev / 12) / 60;

        public static final double maxPower = 0.5;
    }
}
