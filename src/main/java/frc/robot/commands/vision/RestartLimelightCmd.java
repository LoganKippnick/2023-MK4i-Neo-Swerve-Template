package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.WaitCmd;
import frc.robot.subsystems.VisionSys;

public class RestartLimelightCmd extends SequentialCommandGroup {
    
    public RestartLimelightCmd(VisionSys visionSys) {
        super(
            new SetPowerCmd(false, visionSys),
            new WaitCmd(3.0),
            new SetPowerCmd(true, visionSys)
        );
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
    
}
