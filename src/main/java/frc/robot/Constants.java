package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;

public class Constants {

    public enum GameElement {
        kCone,
        kCube,
        kNone
    }

    public static final class CANDevices {

        public static final int powerDistributionHubId = 0;

        public static final int frontLeftSteerEncId = 1;
        public static final int frontLeftSteerMtrId = 9;
        public static final int frontLeftDriveMtrId = 5;

        public static final int frontRightSteerEncId = 2;
        public static final int frontRightSteerMtrId = 10;
        public static final int frontRightDriveMtrId = 6;

        public static final int rearLeftSteerEncId = 3;
        public static final int rearLeftSteerMtrId = 11;
        public static final int rearLeftDriveMtrId = 7;

        public static final int rearRightSteerEncId = 4;
        public static final int rearRightSteerMtrId = 12;
        public static final int rearRightDriveMtrId = 8;

        public static final int imuId = 13;

        public static final int pneumaticHubId = 14;

        public static final int masterMtrId = 15;
        public static final int slaveMtrId = 16;

        public static final int actuationMtrId = 17;
        public static final int upperMtrId = 18;
        public static final int lowerMtrId = 19; 
    }

    public static enum ControllerType {
        kJoystick,
        kGamepad
    }

    public static final class ControllerConstants {

        public static final int driverGamepadPort = 0;

        public static final int driverLeftJoystickPort = 0;
        public static final int driverRightJoystickPort = 1;

        public static final int operatorGamepadPort = 2;

        public static final int hybridControllerPort = 3;

        public static final double joystickDeadband = 0.075;
        public static final double gamepadDeadband = 0.15;
        public static final double triggerPressedDeadband = 0.25;
    }

    public static final class PneumaticChannels {

        public  static final int PCMId = 16;

        public static final int[] clawSolChs = {0, 14};
        
        public static final int[] liftSolChs = {15, 1};

    }

    public static final class DriveConstants {

        /**
         * The track width from wheel center to wheel center.
         */
        public static final double trackWidth = Units.inchesToMeters(24.375);

        /**
         * The track length from wheel center to wheel center.
         */
        public static final double wheelBase = Units.inchesToMeters(24.375);

        /**
         * The SwerveDriveKinematics used for control and odometry.
         */
        public static final SwerveDriveKinematics kinematics = 
            new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2.0, wheelBase / 2.0),  // front left
                new Translation2d(trackWidth / 2.0, -wheelBase / 2.0), // front right
                new Translation2d(-trackWidth / 2.0, wheelBase / 2.0), // rear left
                new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0) // rear right
            );

        /**
         * The gear reduction from the drive motor to the wheel.
         */
        public static final double driveMtrGearReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        /**
         * The gear reduction from the steer motor to the wheel.
         */
        public static final double steerMtrGearReduction = (14.0 / 50.0) * (10.0 / 60.0);

        public static final double wheelRadiusMeters = 0.050686;
        public static final double wheelCircumferenceMeters = 2 * wheelRadiusMeters * Math.PI;

        public static final double driveMetersPerEncRev = wheelCircumferenceMeters * driveMtrGearReduction;
        public static final double driveMetersPerSecPerRPM = driveMetersPerEncRev / 60;

        public static final double steerRadiansPerEncRev = 2 * Math.PI * DriveConstants.steerMtrGearReduction;

        public static final double kFreeMetersPerSecond = 5600 * driveMetersPerSecPerRPM;

        public static final double steerMtrMaxSpeedRadPerSec = 2.0;
        public static final double steerMtrMaxAccelRadPerSecSq = 1.0;

        public static final double maxDriveSpeedMetersPerSec = 5; //3

        public static final double maxTurnRateRadiansPerSec = 2 * Math.PI; //Rate the robot will spin with full Rot command

        public static final double turtleModeSpeedFactor = 0.25;
        public static final double defaultSpeedFactor = 0.5;
        public static final double sprintSpeedFactor = 1.0;

        public static final double frontLeftModOffset = Units.degreesToRadians(284.50 - 180);
        public static final double frontRightModOffset = Units.degreesToRadians(48.42 + 180);
        public static final double rearLeftModOffset = Units.degreesToRadians(176.40 + 180);
        public static final double rearRightModOffset = Units.degreesToRadians(135.97 + 180);

        public static final int driveCurrentLimitAmps = 40;
        public static final int dockCurrentLimitAmps = 50;

        public static final double drivekP = 0.005;

        public static final double steerkP = 1.2477 * 0.3;
        public static final double steerkD = 0.9062 * 0.3;

        public static final double ksVolts = 0.667;
        public static final double kvVoltSecsPerMeter = 2.44;
        public static final double kaVoltSecsPerMeterSq = 0.0;

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(ksVolts, kvVoltSecsPerMeter, kaVoltSecsPerMeterSq);
    }

    public static enum DockDirection {
        kFromCommunity,
        kFromCenter
    }

    public static enum DockHeading {
        kLeft,
        kRight
    }

    public static final class AutoConstants {

        public static final double maxVelMetersPerSec = 3.25;
        public static final double maxAccelMetersPerSecondSq = 3.25;

        public static final double drivekP = 12.6; //12.8
        public static final double drivekD = 0.05;

        public static final double rotkP = 1.25;
        public static final double rotkD = 0.5;

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

        public static final PIDController rotController = constructRotController();
        private static PIDController constructRotController() {
            PIDController rotController = new PIDController(
                rotkP,
                0,
                rotkD
            );

            // Sets wraparound from 0 to 360.
            rotController.enableContinuousInput(0, 360);
            
            return rotController;
        }

        public static final double driveOntoChargeStationVelMetersPerSecond = 0.75;
        public static final double DriveOverChargeStationVelMetersPerSecond = 0.2;

        public static final double onChargeStationDeg = 10.0;
        public static final double chargeStationBalancedToleranceDeg = 13.0;

        public static final double dockVelMetersPerSecond = 0.1;
        
        public static final double dockkP = 0.006;
        public static final double dockkI = 0.0;
        public static final double dockkD = 0.0001;
    }

    public static final class VisionConstants {

        public static final double alignkP = 0.01;
        public static final double alignkD = 0.0;

        public static final PIDController alignController = new PIDController(alignkP, 0.0, alignkD);

        public static final double alignedToleranceDegrees = 2.0;

        public static final double rotkP = 0.0375;
        public static final double rotkD = 0.0045;

        public static final PIDController rotController = constructRotController();
        private static PIDController constructRotController() {
            PIDController rotController = new PIDController(
                rotkP,
                0,
                rotkD
            );

            // Sets wraparound from 0 to 360.
            rotController.enableContinuousInput(0, 360);
            
            return rotController;
        }
    }

    public static final class CompressorConstants {

        public static final double maxPressurePSI = 120.0; // 120 PSI is the legal maximum air tank pressure.
        public static final double minPressurePSI = 80.0;

        public static final double compressorReenableVoltage = 11.5;
        public static final double compressorReenableSeconds = 5.0;
    }
    
    public static final class LiftConstants {

        public static final double gearReduction = 1.0 / 5.0;

        public static final double maxHeightInches = 76.5;
        public static final double inchesPerEncRev = 11 * gearReduction;

        public static final double kP = 0.09;
        public static final double kD = 2.0;

        public static final int maxCurrentAmps = 25;

        public static final double feetPerSecondPerRPM = (inchesPerEncRev / 12) / 60;

        public static final double manualPower = 0.25;
        public static final double downPower = 0.45;
        public static final double hybridPower = 0.55;
        public static final double placeConePower = 0.3;
        public static final double placeCubePower = 0.55;

        public static final double downInches = 0.0;
        public static final double row1Inches = 35.0;
        public static final double row2ShelfInches = 49.0;
        public static final double row2PoleInches = 55.0;
        public static final double row3ShelfInches = 71.0;
        public static final double row3PoleInches = 76.5;

        public static final double manualControlPaddingInches = 2.0;

        public static final double targetToleranceInches = 0.5;

        public static final double downActuationHeightInches = 18.0;
        public static final double upActuationHeightInches = 24.0;
    }

    public class IntakeConstants {

        public static final double rollerGearReduction = 16.0 / 22.0;

        public static final double pulleyDiameterInches = 1.0;
        public static final double beltDiameterInches = pulleyDiameterInches + ((1.0 / 16.0) * 2.0);
        public static final double beltCircumferenceInches = beltDiameterInches * Math.PI;

        public static final double encRevToInches = beltCircumferenceInches;

        public static final double absEncOffset = 0.0;

        public static final int actuationCurrentLimitAmps = 20;
        public static final int rollerCurrentLimitAmps = 50;

        public static final double kP = 3.0;
        public static final double kD = 0.0;

        public static final double manualPower = 0.5;

        public static final double manualControlPadding = 0.5;

        public static final double freeRPM = 5600 * rollerGearReduction;

        public static final double inInches = 2.0;
        public static final double outInches = 15.0;

        public static final double intakeRollerStartInches = 8.0;

        public static final double minInches = 0.0;
        public static final double maxInches = 17.0;

        public static final double targetToleranceInches = 0.5;

        public static final double clawOpenLimitInches = 11.0;

        public static final double rollerWheelDiameterInches = 4.0;
        public static final double rollerWheelCircumferenceInches = rollerWheelDiameterInches * Math.PI;

        public static final double driveMetersPerSecondToRollerRPM = (39.37 * 60.0) / rollerWheelCircumferenceInches;

        public static final double rollerManualControlFactor = 0.8;
        public static final double rollerRelativeMetersPerSecond = 6.5;
        public static final double rollerAbsoluteMetersPerSecond = 6.5;
    }
}
