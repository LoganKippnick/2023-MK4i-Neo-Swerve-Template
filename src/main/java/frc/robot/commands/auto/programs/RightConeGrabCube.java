package frc.robot.commands.auto.programs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.WaitCmd;
import frc.robot.commands.WaitUntilCmd;
import frc.robot.commands.auto.FollowTrajectoryCmd;
import frc.robot.commands.automation.AutoRow3PoleCmd;
import frc.robot.commands.claw.CloseCmd;
import frc.robot.commands.drivetrain.SetHeadingCmd;
import frc.robot.commands.intake.InCmd;
import frc.robot.commands.intake.OutCmd;
import frc.robot.commands.intake.SetRelativeSpeedCmd;
import frc.robot.commands.intake.StopRollersCmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.SwerveSys;

public class RightConeGrabCube extends SequentialCommandGroup {
    
    public RightConeGrabCube(SwerveSys swerveSys, LiftSys liftSys, ClawSys clawSys, IntakeSys intakeSys) {
        super(
            new SetHeadingCmd(new Rotation2d(Math.PI), swerveSys),
            new CloseCmd(clawSys),
            new WaitCmd(0.5),
            new AutoRow3PoleCmd(liftSys, clawSys),
            new FollowTrajectoryCmd("RightStartToCube", swerveSys)
                .alongWith(
                    new WaitUntilCmd(() -> swerveSys.getPose().getX() > 4.5)
                    .andThen(new OutCmd(intakeSys).alongWith(new SetRelativeSpeedCmd(intakeSys, swerveSys)))
                ),
            new WaitCmd(1.0),
            new CloseCmd(clawSys),
            new InCmd(intakeSys).alongWith(new StopRollersCmd(intakeSys))
        );
    }
}
