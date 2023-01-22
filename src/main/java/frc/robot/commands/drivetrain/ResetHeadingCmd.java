package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSys;

public class ResetHeadingCmd extends CommandBase {

    private final DriveSys driveSys;


    public ResetHeadingCmd(DriveSys driveSys) {

        this.driveSys = driveSys;

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        driveSys.zeroGyro();

    }

    @Override
    public boolean isFinished() {

        return true;

    }
    
}
