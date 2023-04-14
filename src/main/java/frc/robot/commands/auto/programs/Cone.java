package frc.robot.commands.auto.programs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.GameElement;
import frc.robot.commands.WaitCmd;
import frc.robot.commands.claw.OpenCmd;
import frc.robot.commands.drivetrain.SetHeadingCmd;
import frc.robot.commands.intake.InCmd;
import frc.robot.commands.lift.DownCmd;
import frc.robot.commands.lift.Row3Cmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.SwerveSys;

public class Cone extends SequentialCommandGroup {
    
    public Cone(SwerveSys swerveSys, LiftSys liftSys, ClawSys clawSys, IntakeSys intakeSys) {
        super(
            new SetHeadingCmd(new Rotation2d(Math.PI), swerveSys),
            new InCmd(intakeSys),
            new Row3Cmd(GameElement.kCone, false, liftSys),
            new WaitCmd(5.0),
            new OpenCmd(clawSys),
            new WaitCmd(2.0),
            new DownCmd(true, liftSys)
        );
    }
}
