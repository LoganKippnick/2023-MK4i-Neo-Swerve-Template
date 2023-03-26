package frc.robot.commands.automation;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.WaitCmd;
import frc.robot.commands.claw.OpenCmd;
import frc.robot.commands.lift.DownCmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.SwerveSys;
import frc.robot.subsystems.VisionSys;
import frc.robot.subsystems.VisionSys.Pipeline;

public class AutoPlaceElementCmd extends SequentialCommandGroup {
    
    public AutoPlaceElementCmd(DoubleSupplier xControl, Pipeline pipeline, VisionSys visionSys, SwerveSys swerveSys, ClawSys clawSys, LiftSys liftSys) {
        // TODO: driver buttons binded to whileTrue
        super(
            new AutoAlignCmd(xControl, pipeline, visionSys, swerveSys, liftSys),
            new OpenCmd(clawSys),
            new WaitCmd(0.5),
            new DownCmd(true, liftSys)
        );
    }
}
