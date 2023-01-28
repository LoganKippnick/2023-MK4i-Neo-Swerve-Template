package frc.robot.commands.lift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LiftSys;

public class SetPowerCmd extends CommandBase { 

    private LiftSys liftSys;

    private double power;

    public SetPowerCmd(double power, LiftSys liftSys) {

        this.liftSys = liftSys;
        this.power = power;

        addRequirements(liftSys);

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        liftSys.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
