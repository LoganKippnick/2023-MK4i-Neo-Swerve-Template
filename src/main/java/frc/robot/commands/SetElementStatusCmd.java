package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GameElement;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.LightsSys;
import frc.robot.subsystems.VisionSys;
import frc.robot.subsystems.VisionSys.Pipeline;

public class SetElementStatusCmd extends CommandBase {

    private final GameElement element;

    private final LiftSys liftSys;
    private final VisionSys visionSys;
    private final LightsSys lightsSys;

    /**
     * Constructs a new ExampleCmd.
     * 
     * <p>ExampleCmd contains the basic framework of a robot command for use in command-based programming.
     * 
     * <p>The command finishes once the isFinished method returns true.
     * 
     * @param exampleSys The required ExampleSys.
     */
    public SetElementStatusCmd(GameElement element, LiftSys liftSys, VisionSys visionSys, LightsSys lightsSys) {

        this.element = element;
        this.liftSys = liftSys;
        this.visionSys = visionSys;
        this.lightsSys = lightsSys;

        addRequirements(visionSys, lightsSys);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        lightsSys.setStatus(element);
        if(element.equals(lightsSys.getStatus())) {
            visionSys.setTarget(element);
        }
        else {
            visionSys.setTarget(GameElement.kNone);
        }
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