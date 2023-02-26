package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitUntilCmd extends CommandBase {

    private final BooleanSupplier condition;

    /**
     * Constructs a new WaitCmd.
     * 
     * <p>WaitCmd is used to wait a specified amount of time for use in sequential commands.
     * 
     * <p>The command finishes once the specified period of time has elapsed.
     * 
     * @param seconds The time to wait, in seconds.
     */
    public WaitUntilCmd(BooleanSupplier condition) {
        this.condition = condition;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return condition.getAsBoolean();
    }

    // Whether the command should run when robot is disabled.
    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}