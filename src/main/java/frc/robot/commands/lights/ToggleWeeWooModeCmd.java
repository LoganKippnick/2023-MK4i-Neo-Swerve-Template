package frc.robot.commands.lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightsSys;

public class ToggleWeeWooModeCmd extends CommandBase {

    private final LightsSys lightsSys;

    public ToggleWeeWooModeCmd(LightsSys lightsSys) {
        this.lightsSys = lightsSys;

        addRequirements(lightsSys);
    }

    @Override
    public void execute() {
        if(lightsSys.isWeeWooMode()) {
            lightsSys.cancelAnimations();
        }
        else {
            lightsSys.setPartyMode(false);
            lightsSys.setWeeWooMode(true);
        }
    }
                
    @Override
    public boolean isFinished() {
        return true;
    }
}
