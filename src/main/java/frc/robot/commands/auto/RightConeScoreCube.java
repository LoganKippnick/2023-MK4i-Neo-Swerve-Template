package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoRow3PoleCmd;
import frc.robot.commands.WaitCmd;
import frc.robot.commands.WaitUntilCmd;
import frc.robot.commands.claw.CloseCmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.SwerveSys;

public class RightConeScoreCube extends SequentialCommandGroup {
    
    public RightConeScoreCube(SwerveSys swerveSys, LiftSys liftSys, ClawSys clawSys, IntakeSys intakeSys) {
        super(
            new CloseCmd(clawSys),
            new AutoRow3PoleCmd(liftSys, clawSys),
            new FollowTrajectoryCmd("RightStartToCube", swerveSys)
                .alongWith(
                    new WaitUntilCmd(() -> swerveSys.getPose().getX() > 4.5)
                    .andThen() // TODO: Start and bring out intake
                ),
            new FollowTrajectoryCmd("RightGrabCubeToScore", swerveSys)
                .alongWith(
                    new WaitCmd(1.0)
                    .andThen(new CloseCmd(clawSys))
                    .andThen() // TODO: Stop and bring in intake
                )
        );
    }
}
