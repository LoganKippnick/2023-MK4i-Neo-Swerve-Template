package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.PneumaticChannels;

public class LiftSys extends SubsystemBase {

    private CANSparkMax masterMtr;
    private CANSparkMax slaveMtr;

    private RelativeEncoder liftEnc;

    private DoubleSolenoid liftSols;

    private SparkMaxPIDController liftController;

    private double targetInches = 24;

    public LiftSys() {
        
        masterMtr = new CANSparkMax(CANDevices.masterMtrId, MotorType.kBrushless);
        slaveMtr = new CANSparkMax(CANDevices.slaveMtrId, MotorType.kBrushless);

        masterMtr.setInverted(false);
        slaveMtr.setInverted(true);

        masterMtr.setSoftLimit(SoftLimitDirection.kForward, LiftConstants.maxHeightInches);
        masterMtr.setSoftLimit(SoftLimitDirection.kReverse, 0);

        masterMtr.enableSoftLimit(SoftLimitDirection.kForward, true);
        masterMtr.enableSoftLimit(SoftLimitDirection.kReverse, true);

        slaveMtr.follow(masterMtr, true);

        liftEnc = masterMtr.getEncoder();

        liftEnc.setPosition(0);

        liftEnc.setPositionConversionFactor(LiftConstants.inchesPerEncRev);
        liftEnc.setVelocityConversionFactor(LiftConstants.feetPerSecondPerRPM);

        liftController = masterMtr.getPIDController();

        liftController.setP(LiftConstants.kP, 0);
        liftController.setD(LiftConstants.kD, 0);

        liftController.setSmartMotionMaxVelocity(LiftConstants.maxSpeedFeetPerSec, 0);
        // TODO: Try acceleration strategy. If still to agressive, try setting a maximum acceleration.
        // liftController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        // liftController.setSmartMotionMaxAccel(LiftConstants.maxAccelFeetPerSecondSq, 0);

        liftSols = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannels.liftSolsCh[0], PneumaticChannels.liftSolsCh[1]);

    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {

        liftController.setReference(targetInches, ControlType.kSmartMotion, 0);

        SmartDashboard.putNumber("lift inches", liftEnc.getPosition());
        SmartDashboard.putNumber("lift power", masterMtr.get());
        SmartDashboard.putNumber("lift target", targetInches);
        SmartDashboard.putNumber("lift velocity", liftEnc.getVelocity());
        
    }

    public void setPower(double power) {
        masterMtr.set(power);
    }

    public void setTarget(double inches) {
        targetInches = inches;
    }
    
}
