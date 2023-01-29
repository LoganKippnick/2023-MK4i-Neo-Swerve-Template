package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSys;

public class ResetHeadingCmd extends CommandBase {

    private final SwerveSys driveSys;


    public ResetHeadingCmd(SwerveSys driveSys) {

        this.driveSys = driveSys;

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        driveSys.resetHeading();

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

        return true;

    }
    
}
