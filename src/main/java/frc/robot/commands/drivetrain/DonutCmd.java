package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSys;

public class DonutCmd extends CommandBase {

    private final SwerveSys swerveSys;

    /**
     * Constructs a new ExampleCmd.
     * 
     * <p>ExampleCmd contains the basic framework of a robot command for use in command-based programming.
     * 
     * <p>The command finishes once the isFinished method returns true.
     * 
     * @param exampleSys The required ExampleSys.
     */
    public DonutCmd(SwerveSys swerveSys) {

        this.swerveSys = swerveSys;

        addRequirements(swerveSys);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    // Versh was here;
    @Override
    public void execute() {
        swerveSys.drive(0.0, 0.0, 1.0, true);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSys.stop();
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