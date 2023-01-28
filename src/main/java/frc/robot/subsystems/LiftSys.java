package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.PneumaticChannels;

public class LiftSys extends SubsystemBase {

    private CANSparkMax masterMtr;
    private CANSparkMax slaveMtr;

    private RelativeEncoder liftEnc;

    private DoubleSolenoid liftSols;

    public LiftSys() {
        masterMtr = new CANSparkMax(CANDevices.masterMtrId, MotorType.kBrushless);
        slaveMtr = new CANSparkMax(CANDevices.slaveMtrId, MotorType.kBrushless);

        masterMtr.setInverted(false);
        slaveMtr.setInverted(true);

        slaveMtr.follow(masterMtr);

        liftEnc = masterMtr.getEncoder();

        liftSols = new DoubleSolenoid(PneumaticsModuleType.REVPH, PneumaticChannels.liftSolsCh[0], PneumaticChannels.liftSolsCh[1]);

    }

    @Override
    public void periodic() {

    }

    public void setPower(double power) {
        masterMtr.set(power);
    }
}
