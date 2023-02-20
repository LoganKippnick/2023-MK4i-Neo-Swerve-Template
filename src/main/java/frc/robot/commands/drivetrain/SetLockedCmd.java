package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSys;

public class SetLockedCmd extends CommandBase {

    private final SwerveSys swerveSys;

    private final boolean isLocked;

    /**
     * Constructs a new SetLockedCmd.
     * 
     * <p>SetLockedCmd is used to set whether the drive base will lock, forming an X-pattern with the wheels.
     * 
     * <p>The command finishes instantly.
     * 
     * @param isLocked The state of locking of the drive base, true if the drive base should lock.
     * @param swerveSys The SwerveSys to modify.
     */
    public SetLockedCmd(boolean isLocked, SwerveSys swerveSys) {

        this.swerveSys = swerveSys;
        this.isLocked = isLocked;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveSys.setLocked(isLocked);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

    // Whether the command should run when robot is disabled.
    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}