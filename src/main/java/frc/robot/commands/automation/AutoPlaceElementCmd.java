package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.claw.OpenCmd;
import frc.robot.commands.lift.DownCmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.SwerveSys;
import frc.robot.subsystems.VisionSys;
import frc.robot.subsystems.VisionSys.TargetType;

public class AutoPlaceElementCmd extends SequentialCommandGroup {
    
    public AutoPlaceElementCmd(TargetType targetType, VisionSys visionSys, SwerveSys swerveSys, ClawSys clawSys, LiftSys liftSys) {
        // TODO: driver buttons binded to whileTrue
        super(
            // TODO: Auto align that finishes when driver is driving into nodes, target is aligned, lift is at target, and lift target is above the actuation threshold.
            new OpenCmd(clawSys),
            new DownCmd(true, liftSys)
        );
    }
}
