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
import frc.robot.commands.claw.CloseCmd;
import frc.robot.commands.drivetrain.SetPoseCmd;
import frc.robot.commands.intake.InCmd;
import frc.robot.commands.intake.OutCmd;
import frc.robot.commands.intake.SetRelativeSpeedCmd;
import frc.robot.commands.intake.StopRollersCmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.SwerveSys;

public class LeftConeGrabCubeDock extends SequentialCommandGroup {
    
    public LeftConeGrabCubeDock(SwerveSys swerveSys, LiftSys liftSys, ClawSys clawSys, IntakeSys intakeSys) {
        super(
            new SetPoseCmd(new Pose2d(1.83, 4.98, new Rotation2d(Math.PI)), swerveSys),
            new CloseCmd(clawSys),
            new InCmd(intakeSys),
            new WaitCmd(0.375),
            new AutoRow3PoleCmd(liftSys, clawSys),
            new FollowTrajectoryCmd("LeftStartToCube", swerveSys)
                .alongWith(
                    new WaitUntilCmd(() -> swerveSys.getPose().getX() > 6.0)
                    .andThen(new OutCmd(intakeSys).alongWith(new SetRelativeSpeedCmd(intakeSys)))
                ),
            new InCmd(intakeSys).alongWith(new StopRollersCmd(intakeSys)),
            new FollowTrajectoryCmd("LeftGrabCubeToDock", swerveSys)
                .alongWith(
                    new InCmd(intakeSys).alongWith(new StopRollersCmd(intakeSys))
                    .andThen(new WaitCmd(1.5))
                    .andThen(new CloseCmd(clawSys))
                ),
            new DockCmd(DockDirection.kFromCenter, swerveSys)
        );
    }
}
