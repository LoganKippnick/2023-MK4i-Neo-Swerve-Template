package frc.robot.commands.lift;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LiftSys;

public class ManualControlCmd extends CommandBase {

    private final LiftSys liftSys;

    private final DoubleSupplier joystick;

    /**
     * Constructs a new ExampleCmd.
     * 
     * <p>ExampleCmd contains the basic framework of a robot command for use in command-based programming.
     * 
     * <p>The command finishes once the isFinished method returns true.
     * 
     * @param exampleSys The required ExampleSys.
     */
    public ManualControlCmd(DoubleSupplier joystick, LiftSys liftSys) {

        this.liftSys = liftSys;
        this.joystick = joystick;

        addRequirements(liftSys);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        liftSys.manualControl(-deadbandInputs(joystick.getAsDouble()));
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

    // method to deadband inputs to eliminate tiny unwanted values from the joysticks
    public double deadbandInputs(double input) {

        if (Math.abs(input) < Constants.Controllers.operatorControllerDeadband)
            return 0.0;
        
        return input;
    }
}