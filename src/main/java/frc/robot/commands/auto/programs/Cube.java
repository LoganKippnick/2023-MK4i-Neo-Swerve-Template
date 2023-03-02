package frc.robot.commands.auto.programs;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.WaitCmd;
import frc.robot.commands.automation.AutoRow3ShelfCmd;
import frc.robot.commands.claw.CloseCmd;
import frc.robot.commands.intake.InCmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.LiftSys;

public class Cube extends SequentialCommandGroup {
    
    public Cube(LiftSys liftSys, ClawSys clawSys, IntakeSys intakeSys) {
        super(
            new CloseCmd(clawSys),
            new InCmd(intakeSys),
            new WaitCmd(0.5),
            new AutoRow3ShelfCmd(liftSys, clawSys)
        );
    }
}
