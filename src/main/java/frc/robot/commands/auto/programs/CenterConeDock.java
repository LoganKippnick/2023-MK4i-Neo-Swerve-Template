package frc.robot.commands.auto.programs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DockDirection;
import frc.robot.commands.auto.DockCmd;
import frc.robot.commands.auto.FollowTrajectoryCmd;
import frc.robot.commands.automation.AutoRow3PoleCmd;
import frc.robot.commands.drivetrain.SetPoseCmd;
import frc.robot.commands.intake.InCmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.LightsSys;
import frc.robot.subsystems.SwerveSys;

public class CenterConeDock extends SequentialCommandGroup {
    
    public CenterConeDock(SwerveSys swerveSys, LiftSys liftSys, ClawSys clawSys, IntakeSys intakeSys, LightsSys lightsSys) {
        super(
            new SetPoseCmd(new Pose2d(1.83, 2.74, new Rotation2d(Math.PI)), swerveSys),
            new InCmd(intakeSys),
            new AutoRow3PoleCmd(liftSys, clawSys),
            new FollowTrajectoryCmd("CenterStartToDock", 0.2, 0.25, swerveSys),
            new DockCmd(DockDirection.kFromCommunity, swerveSys, lightsSys)
        );
    }
}
