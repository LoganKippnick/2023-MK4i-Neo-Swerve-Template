package frc.robot.commands.auto.programs;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.automation.AutoRow3PoleCmd;
import frc.robot.commands.intake.InCmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.LiftSys;

public class Cone extends SequentialCommandGroup {
    
    public Cone(LiftSys liftSys, ClawSys clawSys, IntakeSys intakeSys) {
        super(
            new InCmd(intakeSys),
            new AutoRow3PoleCmd(liftSys, clawSys)
        );
    }
}
