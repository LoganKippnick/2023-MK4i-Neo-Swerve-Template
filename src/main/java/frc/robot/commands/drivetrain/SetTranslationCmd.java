package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSys;

public class SetTranslationCmd extends CommandBase {

    private final SwerveSys swerveSys;

    private final Translation2d translation;

    public SetTranslationCmd(Translation2d translation, SwerveSys swerveSys) {
        this.swerveSys = swerveSys;
        this.translation = translation;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        swerveSys.setPose(new Pose2d(translation, swerveSys.getHeading()));
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

        return true;

    }
    
}
