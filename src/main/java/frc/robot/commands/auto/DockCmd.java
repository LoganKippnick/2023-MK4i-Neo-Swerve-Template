package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DockDirection;
import frc.robot.subsystems.SwerveSys;

public class DockCmd extends CommandBase {

    private final SwerveSys swerveSys;

    private final DockDirection direction;
    // private final DockHeading heading;

    private boolean onChargeStation = false;

    private final PIDController dockController;

    /**
     * Constructs a new ExampleCmd.
     * 
     * <p>ExampleCmd contains the basic framework of a robot command for use in command-based programming.
     * 
     * <p>The command finishes once the isFinished method returns true.
     * 
     * @param exampleSys The required ExampleSys.
     */
    public DockCmd(DockDirection direction, SwerveSys swerveSys) {

        this.swerveSys = swerveSys;
        this.direction = direction;

        dockController = new PIDController(
            AutoConstants.dockkP,
            AutoConstants.dockkI,
            AutoConstants.dockkD
        );
        dockController.setSetpoint(0.0);

        addRequirements(swerveSys);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(Math.abs(swerveSys.getRollDegrees()) > AutoConstants.onChargeStationDeg) {
            onChargeStation = true;
        }

        if(!onChargeStation) {
            swerveSys.setLocked(false);
            swerveSys.drive(
                AutoConstants.driveOntoChargeStationVelMetersPerSecond * (direction.equals(DockDirection.kFromCenter) ? -1 : 1),
                0.0,
                0.0,
                true
            );
        }
        else if(Math.abs(swerveSys.getRollDegrees()) < AutoConstants.chargeStationBalancedToleranceDeg) {
            swerveSys.setLocked(true);
        }
        else {
            // double dockVel = dockController.calculate(swerveSys.getRoll());
            double dockVel = AutoConstants.dockVelMetersPerSecond;
            // if(Math.abs(dockVel) > AutoConstants.dockMaxVelMetersPerSecond) {
            //     dockVel = Math.copySign(AutoConstants.dockMaxVelMetersPerSecond, dockVel);
            // }
            if(direction.equals(DockDirection.kFromCenter)) {
                dockVel *= -1;
            }

            swerveSys.setLocked(false);
            swerveSys.drive(
                dockVel,
                0.0,
                0.0,
                true
            );
        }
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSys.setLocked(false);
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Whether the command should run when robot is disabled.
    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}