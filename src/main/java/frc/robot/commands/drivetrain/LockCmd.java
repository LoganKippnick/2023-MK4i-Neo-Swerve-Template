package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSys;

public class LockCmd extends CommandBase {

    private final SwerveSys swerveSys;

    /**
     * Constructs a new SetLockedCmd.
     * 
     * <p>SetLockedCmd is used to set whether the drive base will lock, forming an X-pattern with the wheels.
     * 
     * <p>The command finishes instantly.
     * 
     * @param swerveSys The SwerveSys to modify.
     */
    public LockCmd(SwerveSys swerveSys) {
        this.swerveSys = swerveSys;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveSys.lock();
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Whether the command should run when robot is disabled.
    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}