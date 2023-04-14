package frc.robot.commands.lift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GameElement;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.LightsSys;

public class Row2Cmd extends CommandBase {

    private GameElement element = GameElement.kNone;

    private final LiftSys liftSys;

    private final boolean finishInstantly;

    /**
     * Constructs a new ExampleCmd.
     * 
     * <p>ExampleCmd contains the basic framework of a robot command for use in command-based programming.
     * 
     * <p>The command finishes once the isFinished method returns true.
     * 
     * @param exampleSys The required ExampleSys.
     */
    public Row2Cmd(boolean finishInstantly, LiftSys liftSys) {
        this.liftSys = liftSys;
        this.finishInstantly = finishInstantly;

        addRequirements(liftSys);
    }

    public Row2Cmd(GameElement element, boolean finishInstantly, LiftSys liftSys) {
        this(finishInstantly, liftSys);
        this.element = element;
    }

    public Row2Cmd(LightsSys lightsSys, boolean finishInstantly, LiftSys liftSys) {
        this(finishInstantly, liftSys);
        this.element = lightsSys.getStatus();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        liftSys.setArticulationOverride(false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(element.equals(GameElement.kCube))
            liftSys.setTarget(LiftConstants.row2ShelfInches, LiftConstants.placeCubePower);
        else
            liftSys.setTarget(LiftConstants.row2PoleInches, LiftConstants.placeConePower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finishInstantly || liftSys.isAtTarget();
    }

    // Whether the command should run when robot is disabled.
    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}