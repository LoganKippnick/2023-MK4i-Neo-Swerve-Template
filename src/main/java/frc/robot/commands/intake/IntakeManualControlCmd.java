package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSys;

public class IntakeManualControlCmd extends CommandBase {

    private final IntakeSys intakeSys;

    private final DoubleSupplier actuationJoystick;
    private final DoubleSupplier intakeTrigger;
    private final DoubleSupplier outtakeTrigger;

    /**
     * Constructs a new ExampleCmd.
     * 
     * <p>ExampleCmd contains the basic framework of a robot command for use in command-based programming.
     * 
     * <p>The command finishes once the isFinished method returns true.
     * 
     * @param exampleSys The required ExampleSys.
     */
    public IntakeManualControlCmd(
        DoubleSupplier actuationJoystick,
        DoubleSupplier intakeTrigger,
        DoubleSupplier outtakeTrigger,
        IntakeSys intakeSys
    ) {

        this.intakeSys = intakeSys;
        this.actuationJoystick = actuationJoystick;
        this.intakeTrigger = intakeTrigger;
        this.outtakeTrigger = outtakeTrigger;

        addRequirements(intakeSys);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("actuationJoystick", actuationJoystick.getAsDouble());
        intakeSys.manualActuationControl(-actuationJoystick.getAsDouble());

        intakeSys.manualRollerControl(intakeTrigger.getAsDouble() - outtakeTrigger.getAsDouble());
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