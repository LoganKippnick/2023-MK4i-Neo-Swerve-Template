package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GameElement;
import frc.robot.commands.WaitCmd;
import frc.robot.commands.claw.OpenCmd;
import frc.robot.commands.lift.DownCmd;
import frc.robot.commands.lift.Row3Cmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.LiftSys;

public class AutoRow3PoleCmd extends SequentialCommandGroup {
    
    public AutoRow3PoleCmd(LiftSys liftSys, ClawSys clawSys) {
        super(
            new Row3Cmd(GameElement.kCube, false, liftSys),
            new WaitCmd(0.25),
            new OpenCmd(clawSys),
            new WaitCmd(0.25),
            new DownCmd(true, liftSys)
        );
    }

}
