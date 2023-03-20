package frc.robot.commands.lift;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.LiftSys;

public class DownCmd extends CommandBase {

    private final LiftSys liftSys;

    private final boolean finishInstantly;

    private final Timer timer;

    private boolean isWaiting;

    /**
     * Constructs a new ExampleCmd.
     * 
     * <p>ExampleCmd contains the basic framework of a robot command for use in command-based programming.
     * 
     * <p>The command finishes once the isFinished method returns true.
     * 
     * @param exampleSys The required ExampleSys.
     */
    public DownCmd(boolean finishInstantly, LiftSys liftSys) {
        this.liftSys = liftSys;
        this.finishInstantly = finishInstantly;

        timer = new Timer();

        addRequirements(liftSys);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(liftSys.getCurrentPosition() < LiftConstants.row1Inches + 6.0) {
            isWaiting = true;
        }
        else
            isWaiting = false;

        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(!isWaiting || timer.hasElapsed(0.75))
            liftSys.setTarget(LiftConstants.downInches, LiftConstants.downPower);
        else
            liftSys.setTarget(LiftConstants.downActuationHeightInches - LiftConstants.targetTolerance, LiftConstants.downPower);
    }

    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (liftSys.getTargetInches() == LiftConstants.downInches) || (!finishInstantly && (
            liftSys.getCurrentPosition() >= LiftConstants.downInches - LiftConstants.targetTolerance &&
            liftSys.getCurrentPosition() <= LiftConstants.downInches + LiftConstants.targetTolerance
        ));
    }

    // Whether the command should run when robot is disabled.
    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}