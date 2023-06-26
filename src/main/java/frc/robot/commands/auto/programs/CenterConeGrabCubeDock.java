package frc.robot.commands.auto.programs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DockDirection;
import frc.robot.commands.WaitCmd;
import frc.robot.commands.auto.DockCmd;
import frc.robot.commands.auto.DriveOverChargeStationCmd;
import frc.robot.commands.auto.FollowTrajectoryCmd;
import frc.robot.commands.automation.AutoRow3PoleCmd;
import frc.robot.commands.drivetrain.SetPoseCmd;
import frc.robot.commands.drivetrain.SetTranslationCmd;
import frc.robot.commands.intake.InCmd;
import frc.robot.commands.intake.OutCmd;
import frc.robot.commands.intake.IntakeCubeCmd;
import frc.robot.commands.intake.StopRollersCmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.LightsSys;
import frc.robot.subsystems.SwerveSys;

public class CenterConeGrabCubeDock extends SequentialCommandGroup {
    
    public CenterConeGrabCubeDock(SwerveSys swerveSys, LiftSys liftSys, ClawSys clawSys, IntakeSys intakeSys, LightsSys lightsSys) {
        super(
            new SetPoseCmd(new Pose2d(1.83, 2.74, new Rotation2d(Math.PI)), swerveSys),
            new InCmd(intakeSys),
            new AutoRow3PoleCmd(liftSys, clawSys),
            new FollowTrajectoryCmd("CenterStartToDock", 0.25, 0.25, swerveSys),
            new DriveOverChargeStationCmd(DockDirection.kFromCommunity, swerveSys),
            new SetTranslationCmd(new Translation2d(5.50, 2.74), swerveSys),
            new FollowTrajectoryCmd("CenterGrabCubeToDock", 3.375, 2.5, swerveSys).alongWith(
                new OutCmd(intakeSys, lightsSys).alongWith(new IntakeCubeCmd(intakeSys, lightsSys))
                .andThen(new WaitCmd(1.5))
                .andThen(new InCmd(intakeSys).alongWith(new StopRollersCmd(intakeSys, lightsSys)))
            ),
            new DockCmd(DockDirection.kFromCenter, swerveSys, lightsSys)
        );
    }
}
