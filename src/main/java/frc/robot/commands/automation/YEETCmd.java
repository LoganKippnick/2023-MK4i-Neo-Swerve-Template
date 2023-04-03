package frc.robot.commands.automation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.LiftSys;

public class YEETCmd extends CommandBase {
    
    private final LiftSys liftSys;
    private final ClawSys clawSys;

    private final Timer holdTimer;

    public YEETCmd(LiftSys liftSys, ClawSys clawSys) {
        this.liftSys = liftSys;
        this.clawSys = clawSys;

        holdTimer = new Timer();

        addRequirements(liftSys, clawSys);
    }

    @Override
    public void initialize() {
        if(clawSys.isOpen()){
            cancel();
        }
        else {
            liftSys.setArticulationOverride(false);
            liftSys.setTarget(LiftConstants.YEETHeightInches, LiftConstants.hybridPower);
        }
    }

    @Override
    public void execute() {
        if(liftSys.getCurrentPosition() >= LiftConstants.YEETReleaseInches) {
            clawSys.open();
        }

        if(liftSys.isAtTarget() && liftSys.getTargetInches() == LiftConstants.YEETHeightInches) {
            holdTimer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        liftSys.setTarget(LiftConstants.downInches, LiftConstants.downPower);
        holdTimer.stop();
        holdTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return holdTimer.hasElapsed(0.25);
    }
}
