package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSys;

public class DriveTime extends CommandBase {

    private final SwerveSys drive;

    private double driveSeconds;
    private double drivePower;

    private Timer timer = new Timer();

    public DriveTime(double seconds, double power, SwerveSys subsystem) {

        driveSeconds = seconds;
        drivePower = power;

        drive = subsystem;

    }

    @Override
    public void initialize() {

        timer.start();
        timer.reset();

    }

    @Override
    public void execute() {

        drive.drive(drivePower, 0, 0, false, false);

    }

    @Override
    public void end(boolean interrupted) {

        drive.drive(0, 0, 0, false, false);

    }

    @Override
    public boolean isFinished() {

        return timer.get() >= driveSeconds;

    }
    
}
