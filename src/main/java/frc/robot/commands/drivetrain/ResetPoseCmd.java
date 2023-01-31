package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSys;

public class ResetPoseCmd extends CommandBase {

    private final SwerveSys swerveSys;


    public ResetPoseCmd(SwerveSys swerveSys) {

        this.swerveSys = swerveSys;

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        swerveSys.resetPose();

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

        return true;

    }
    
}
