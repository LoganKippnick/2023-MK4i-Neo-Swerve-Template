package frc.robot.commands.auto.programs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DockDirection;
import frc.robot.commands.WaitCmd;
import frc.robot.commands.WaitUntilCmd;
import frc.robot.commands.auto.DockCmd;
import frc.robot.commands.auto.FollowTrajectoryCmd;
import frc.robot.commands.automation.AutoRow3PoleCmd;
import frc.robot.commands.drivetrain.SetPoseCmd;
import frc.robot.commands.intake.InCmd;
import frc.robot.commands.intake.OutCmd;
import frc.robot.commands.intake.SetAbsoluteSpeedCmd;
import frc.robot.commands.intake.StopRollersCmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.LightsSys;
import frc.robot.subsystems.SwerveSys;

public class RightConeGrabCubeDock extends SequentialCommandGroup {
    
    public RightConeGrabCubeDock(SwerveSys swerveSys, LiftSys liftSys, ClawSys clawSys, IntakeSys intakeSys, LightsSys lightsSys) {
        super(
            new SetPoseCmd(new Pose2d(1.83, 0.5, new Rotation2d(Math.PI)), swerveSys),
            new InCmd(intakeSys),
            new AutoRow3PoleCmd(liftSys, clawSys),
            new SetPoseCmd(new Pose2d(1.83, 0.5, new Rotation2d(Math.PI)), swerveSys),
            new FollowTrajectoryCmd("RightStartToGrabCube1Dock", swerveSys).alongWith(
                new WaitUntilCmd(() -> swerveSys.getPose().getX() > 5.75)
                .andThen(new OutCmd(intakeSys, lightsSys).alongWith(new SetAbsoluteSpeedCmd(intakeSys, lightsSys)))
                .andThen(new WaitCmd(1.0))
                .andThen(new InCmd(intakeSys).alongWith(new StopRollersCmd(intakeSys, lightsSys)))
            ),
            new DockCmd(DockDirection.kFromCenter, swerveSys, lightsSys)
        );
    }
}
