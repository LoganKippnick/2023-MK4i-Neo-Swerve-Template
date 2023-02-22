package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DockDirection;
import frc.robot.subsystems.SwerveSys;

public class DriveOverChargeStationCmd extends CommandBase {

    private final SwerveSys swerveSys;

    private final DockDirection direction;

    private final PIDController strafeController;
    private final ProfiledPIDController rotController;

    private boolean isOverCenter = false;

    private final Timer offChargeStationTimer;

    /**
     * Constructs a new ExampleCmd.
     * 
     * <p>ExampleCmd contains the basic framework of a robot command for use in command-based programming.
     * 
     * <p>The command finishes once the isFinished method returns true.
     * 
     * @param exampleSys The required ExampleSys.
     */
    public DriveOverChargeStationCmd(DockDirection direction, SwerveSys swerveSys) {

        this.swerveSys = swerveSys;
        this.direction = direction;

        offChargeStationTimer = new Timer();

        strafeController = AutoConstants.driveController;
        strafeController.setSetpoint(0.0);

        rotController = AutoConstants.rotController;
        rotController.setGoal(0.0);

        addRequirements(swerveSys);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if(
            (direction.equals(DockDirection.kFromCommunity) && swerveSys.getRoll() < -AutoConstants.onChargeStationDeg) ||
            (direction.equals(DockDirection.kFromCommunity) && swerveSys.getRoll() > AutoConstants.onChargeStationDeg)
        ) {
            isOverCenter = true;
        }

        swerveSys.drive(
            AutoConstants.driveOntoChargeStationVelMetersPerSecond * (direction.equals(DockDirection.kFromCenter) ? -1 : 1),
            strafeController.calculate(swerveSys.getPose().getY()),
            rotController.calculate(swerveSys.getHeading().getRadians()),
            true
        );

        if(isOverCenter && Math.abs(swerveSys.getPitch()) < AutoConstants.chargeStationBalancedToleranceDeg) {
            offChargeStationTimer.start();
        }

    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSys.stop();
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return offChargeStationTimer.hasElapsed(1.0);
    }

    // Whether the command should run when robot is disabled.
    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}