package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSys;

public class DefaultSpeedCmd extends CommandBase {

    private final SwerveSys swerveSys;

    /**
     * Constructs a new DefaultSpeedCmd.
     * 
     * <p>DefaultSpeedCmd is used to reduce the speed factor of the drive base for better precision when driving.
     * 
     * <p>The command finishes instantly.
     * 
     * @param swerveSys The SwerveSys to modify.
     */
    public DefaultSpeedCmd(SwerveSys swerveSys) {

        this.swerveSys = swerveSys;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveSys.setSpeedFactor(DriveConstants.defaultSpeedFactor);
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