package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSys;

public class ResetPoseCmd extends CommandBase {

    private final SwerveSys driveSys;


    public ResetPoseCmd(SwerveSys driveSys) {

        this.driveSys = driveSys;

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        driveSys.setPose(new Pose2d());

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

        return true;

    }
    
}
