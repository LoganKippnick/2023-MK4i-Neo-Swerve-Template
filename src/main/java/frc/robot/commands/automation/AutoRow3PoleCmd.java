package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.WaitCmd;
import frc.robot.commands.claw.OpenCmd;
import frc.robot.commands.lift.DownCmd;
import frc.robot.commands.lift.Row3PoleCmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.LiftSys;

public class AutoRow3PoleCmd extends SequentialCommandGroup {
    
    public AutoRow3PoleCmd(LiftSys liftSys, ClawSys clawSys) {
        super(
            new Row3PoleCmd(false, liftSys),
            new WaitCmd(0.25),
            new OpenCmd(clawSys),
            new WaitCmd(0.25),
            new DownCmd(true, liftSys)
        );
    }

}
