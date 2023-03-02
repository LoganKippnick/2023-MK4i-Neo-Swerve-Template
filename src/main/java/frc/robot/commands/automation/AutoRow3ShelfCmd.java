package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.WaitCmd;
import frc.robot.commands.claw.OpenCmd;
import frc.robot.commands.lift.DownCmd;
import frc.robot.commands.lift.Row3ShelfCmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.LiftSys;

public class AutoRow3ShelfCmd extends SequentialCommandGroup {
    
    public AutoRow3ShelfCmd(LiftSys liftSys, ClawSys clawSys) {
        super(
            new Row3ShelfCmd(false, liftSys),
            new WaitCmd(0.5),
            new OpenCmd(clawSys),
            new WaitCmd(0.5),
            new DownCmd(false, liftSys)
        );
    }

}
