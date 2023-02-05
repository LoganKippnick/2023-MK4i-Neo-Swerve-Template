package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
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

    private SparkMaxPIDController controller;

    private DoubleSolenoid liftSols;

    private double targetInches = 0;

    public LiftSys() {
        masterMtr = new CANSparkMax(CANDevices.masterMtrId, MotorType.kBrushless);
        slaveMtr = new CANSparkMax(CANDevices.slaveMtrId, MotorType.kBrushless);

        masterMtr.setInverted(false);
        slaveMtr.setInverted(true);

        masterMtr.setSmartCurrentLimit(LiftConstants.maxCurrentAmps);

        slaveMtr.follow(masterMtr, true);

        liftEnc = masterMtr.getEncoder();

        liftEnc.setPosition(0);

        liftEnc.setPositionConversionFactor(LiftConstants.inchesPerEncRev);
        liftEnc.setVelocityConversionFactor(LiftConstants.feetPerSecondPerRPM);

        controller = masterMtr.getPIDController();

        controller.setP(LiftConstants.kP);
        controller.setD(LiftConstants.kD);

        controller.setOutputRange(LiftConstants.minPower, LiftConstants.maxPower);
        
        controller.setIZone(0);
        
        liftSols = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannels.liftSolsCh[0], PneumaticChannels.liftSolsCh[1]);

    }

    @Override
    public void periodic() {

        controller.setReference(targetInches, ControlType.kPosition);

        SmartDashboard.putNumber("lift inches", liftEnc.getPosition());
        SmartDashboard.putNumber("lift velocity", liftEnc.getVelocity());
        SmartDashboard.putNumber("lift target", targetInches);
        SmartDashboard.putNumber("lift power", masterMtr.get());

    }

    public void setPower(double power) {
        masterMtr.set(power);
    }

    public void setTarget(double inches) {
        if(inches > LiftConstants.maxHeightInches) inches = LiftConstants.maxHeightInches;

        targetInches = inches;
    }
}