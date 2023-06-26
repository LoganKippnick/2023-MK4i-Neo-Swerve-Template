package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GameElement;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.LightsSys;
import frc.robot.subsystems.VisionSys;

public class SetElementStatusCmd extends CommandBase {

    private final GameElement element;

    private final LiftSys liftSys;
    private final IntakeSys intakeSys;
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
    public SetElementStatusCmd(GameElement element, LiftSys liftSys, IntakeSys intakeSys, VisionSys visionSys, LightsSys lightsSys) {

        this.element = element;
        this.liftSys = liftSys;
        this.intakeSys = intakeSys;
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
        if(!element.equals(GameElement.kNone)) {
            lightsSys.blink();
        }

        visionSys.setTarget(element);
        lightsSys.setStatus(element);

        if(element.equals(GameElement.kCone)) {
            if(liftSys.getTargetInches() == LiftConstants.row2ShelfInches) {
                liftSys.setTarget(LiftConstants.row2PoleInches, LiftConstants.placeConePower);
            }
            else if(liftSys.getTargetInches() == LiftConstants.row3ShelfInches) {
                liftSys.setTarget(LiftConstants.row3PoleInches, LiftConstants.placeConePower);
            }

            if(intakeSys.getTargetInches() == IntakeConstants.outInches) {
                intakeSys.setTarget(IntakeConstants.coneInches);
            }
        }
        else if(element.equals(GameElement.kCube)) {
            if(liftSys.getTargetInches() == LiftConstants.row2PoleInches) {
                liftSys.setTarget(LiftConstants.row2ShelfInches, LiftConstants.placeCubePower);
            }
            else if(liftSys.getTargetInches() == LiftConstants.row3PoleInches) {
                liftSys.setTarget(LiftConstants.row3ShelfInches, LiftConstants.placeCubePower);
            }

            if(intakeSys.getTargetInches() == IntakeConstants.coneInches) {
                intakeSys.setTarget(IntakeConstants.outInches);
            }
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