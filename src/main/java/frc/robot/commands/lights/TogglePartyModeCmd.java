package frc.robot.commands.lights;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightsSys;

public class TogglePartyModeCmd extends CommandBase {

    private final LightsSys lightsSys;

    public TogglePartyModeCmd(LightsSys lightsSys) {
        this.lightsSys = lightsSys;

        addRequirements(lightsSys);
    }

    @Override
    public void execute() {
        if(lightsSys.isPartyMode()) {
            lightsSys.cancelAnimations();
        }
        else {
            lightsSys.setPartyMode(true);
            lightsSys.setWeeWooMode(false);
        }
    }
                
    @Override
    public boolean isFinished() {
        return true;
    }
}
