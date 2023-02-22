package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.PneumaticChannels;

public class ClawSys extends SubsystemBase {

    private final DoubleSolenoid clawSol;

    public ClawSys() {
        // pcm = new PneumaticsControlModule(CANDevices.pneumaticHubId);
        clawSol = new DoubleSolenoid(CANDevices.pneumaticHubId, PneumaticsModuleType.REVPH, PneumaticChannels.clawSolChs[0], PneumaticChannels.clawSolChs[1]);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {

        if(clawSol.get().equals(Value.kForward)) SmartDashboard.putString("claw status", "open");
        else SmartDashboard.putString("claw status", "close");

        
    }
    
    public void open() {
        clawSol.set(Value.kForward);
    }

    public void close() {
        clawSol.set(Value.kReverse);
    }
}