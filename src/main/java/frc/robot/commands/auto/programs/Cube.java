package frc.robot.commands.auto.programs;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.automation.AutoRow3ShelfCmd;
import frc.robot.commands.intake.InCmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.LiftSys;

public class Cube extends SequentialCommandGroup {
    
    public Cube(LiftSys liftSys, ClawSys clawSys, IntakeSys intakeSys) {
        super(
            new InCmd(intakeSys),
            new AutoRow3ShelfCmd(liftSys, clawSys)
        );
    }
}
