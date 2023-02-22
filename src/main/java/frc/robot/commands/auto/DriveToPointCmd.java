package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSys;

public class DriveToPointCmd extends CommandBase {

    private final SwerveSys swerveSys;

    private final Pose2d target;

    private final double maxVelMetersPerSec;

    private final boolean stopAtTarget;

    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController rotController;

    private double xVel;
    private double yVel;
    private double rotVel;

    /**
     * Constructs a new DriveToPointCmd.
     * 
     * <p> Drives to a specified point.
     * 
     * @param target The target Pose2d.
     * @param maxVelMetersPerSec The maximum velocity measured in meters per second.
     * @param stopAtTarget If true, the robot will come to a complete stop at the target Pose2d.
     * @param swerveSys The required SwerveSys.
     */
    public DriveToPointCmd(Pose2d target, double maxVelMetersPerSec, boolean stopAtTarget, SwerveSys swerveSys) {

        this.swerveSys = swerveSys;
        this.target = target;
        this.maxVelMetersPerSec = maxVelMetersPerSec;
        this.stopAtTarget = stopAtTarget;

        xController = AutoConstants.driveController;
        yController = AutoConstants.driveController;
        rotController = AutoConstants.rotController;

        addRequirements(swerveSys);
    }



    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        xController.setSetpoint(target.getX());
        yController.setSetpoint(target.getY());
        rotController.setGoal(target.getRotation().getRadians());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        xVel = xController.calculate(swerveSys.getPose().getX()) * DriveConstants.maxDriveSpeedMetersPerSec;
        yVel = yController.calculate(swerveSys.getPose().getY()) * DriveConstants.maxDriveSpeedMetersPerSec;
        rotVel = rotController.calculate(swerveSys.getHeading().getRadians()) * DriveConstants.maxTurnRateRadiansPerSec;

        if(xVel < maxVelMetersPerSec) xVel = maxVelMetersPerSec;
        if(yVel < maxVelMetersPerSec) yVel = maxVelMetersPerSec;

        swerveSys.drive(xVel, yVel, rotVel, true);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSys.stop();
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(stopAtTarget)
            return Math.abs(swerveSys.getAverageDriveVelocityMetersPerSecond()) < 0.1;
        else
            return Math.abs(xVel) < 0.1 && Math.abs(yVel) < 0.1 && Math.abs(rotVel) < 0.1;
    }

    // Whether the command should run when robot is disabled.
    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}