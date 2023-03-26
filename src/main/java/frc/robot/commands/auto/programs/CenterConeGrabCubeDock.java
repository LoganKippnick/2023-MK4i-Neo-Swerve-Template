package frc.robot.commands.auto.programs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DockDirection;
import frc.robot.commands.WaitCmd;
import frc.robot.commands.auto.DockCmd;
import frc.robot.commands.auto.DriveOverChargeStationCmd;
import frc.robot.commands.auto.FollowTrajectoryCmd;
import frc.robot.commands.automation.AutoRow3PoleCmd;
import frc.robot.commands.claw.CloseCmd;
import frc.robot.commands.drivetrain.SetPoseCmd;
import frc.robot.commands.intake.InCmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.SwerveSys;

public class CenterConeGrabCubeDock extends SequentialCommandGroup {
    
    public CenterConeGrabCubeDock(SwerveSys swerveSys, LiftSys liftSys, ClawSys clawSys, IntakeSys intakeSys) {
        super(
            new SetPoseCmd(new Pose2d(1.83, 2.74, new Rotation2d(Math.PI)), swerveSys),
            new CloseCmd(clawSys),
            new InCmd(intakeSys),
            new WaitCmd(0.5),
            new AutoRow3PoleCmd(liftSys, clawSys),
            new FollowTrajectoryCmd("CenterStartToDock", 0.5, 1.0, swerveSys),
            new DriveOverChargeStationCmd(DockDirection.kFromCommunity, swerveSys),
            new WaitCmd(0.5),
            new SetPoseCmd(new Pose2d(5.50, 2.74, swerveSys.getHeading()), swerveSys),
            new FollowTrajectoryCmd("CenterGrabCubeToDock", 0.5, 1.0, swerveSys),
            new DockCmd(DockDirection.kFromCenter, swerveSys)
        );
    }
}
