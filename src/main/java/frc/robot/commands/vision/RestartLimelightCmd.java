package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CANDevices;
import frc.robot.commands.WaitCmd;

public class RestartLimelightCmd extends SequentialCommandGroup {

    private static final PowerDistribution pdh = new PowerDistribution(CANDevices.powerDistributionHubId, ModuleType.kRev);
    
    public RestartLimelightCmd() {
        super(
            new RunCommand(() -> pdh.setSwitchableChannel(false)),
            new WaitCmd(3.0),
            new RunCommand(() -> pdh.setSwitchableChannel(true))
        );
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
    
}
