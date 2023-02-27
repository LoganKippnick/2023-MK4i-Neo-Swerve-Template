package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.IntakeSys;

public class OutCmd extends CommandBase {

    private final IntakeSys intakeSys;

    /**
     * Constructs a new ExampleCmd.
     * 
     * <p>ExampleCmd contains the basic framework of a robot command for use in command-based programming.
     * 
     * <p>The command finishes once the isFinished method returns true.
     * 
     * @param exampleSys The required ExampleSys.
     */
    public OutCmd(IntakeSys intakeSys) {
        this.intakeSys = intakeSys;

        addRequirements(intakeSys);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSys.setTarget(IntakeConstants.outInches);
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