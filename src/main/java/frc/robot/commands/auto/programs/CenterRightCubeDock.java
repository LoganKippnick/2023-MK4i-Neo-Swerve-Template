package frc.robot.commands.auto.programs;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DockDirection;
import frc.robot.commands.WaitCmd;
import frc.robot.commands.auto.DockCmd;
import frc.robot.commands.auto.FollowTrajectoryCmd;
import frc.robot.commands.automation.AutoRow3ShelfCmd;
import frc.robot.commands.claw.CloseCmd;
import frc.robot.commands.intake.InCmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.SwerveSys;

public class CenterRightCubeDock extends SequentialCommandGroup {
    
    public CenterRightCubeDock(SwerveSys swerveSys, LiftSys liftSys, ClawSys clawSys, IntakeSys intakeSys) {
        super(
            // FIXME: Set pose
            new CloseCmd(clawSys),
            new InCmd(intakeSys),
            new WaitCmd(0.5),
            new AutoRow3ShelfCmd(liftSys, clawSys),
            new FollowTrajectoryCmd("CenterRightStartToDock", 0.5, 1.0, swerveSys),
            new DockCmd(DockDirection.kFromCommunity, swerveSys)
        );
    }
}
