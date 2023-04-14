package frc.robot.commands.automation;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GameElement;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.SwerveSys;
import frc.robot.subsystems.VisionSys;

public class AutoAlignCmd extends CommandBase {

    private final VisionSys visionSys;
    private final SwerveSys swerveSys;

    private final DoubleSupplier xControl;
    private final DoubleSupplier yControl;

    private final PIDController alignController;
    private final PIDController rotController;
    
    public AutoAlignCmd(DoubleSupplier xControl, DoubleSupplier yControl, VisionSys visionSys, SwerveSys swerveSys, LiftSys liftSys) {
        this.visionSys = visionSys;
        this.swerveSys = swerveSys;
        this.xControl = xControl;
        this.yControl = yControl;

        alignController = visionSys.getPipeline().equals(GameElement.kCone) ? VisionConstants.coneController : VisionConstants.cubeController;
        alignController.setSetpoint(0.0);

        rotController = VisionConstants.rotController;
        rotController.setSetpoint(180);

        addRequirements(visionSys, swerveSys);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        swerveSys.setIsTracking(true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double rot;
        if(Math.abs(swerveSys.getHeading().getDegrees() - rotController.getSetpoint()) < VisionConstants.rotToleranceDeg)
            rot = 0.0;
        else
            rot = rotController.calculate(swerveSys.getHeading().getDegrees());

        double strafe;
        if(!visionSys.hasTarget()) {
            strafe = yControl.getAsDouble() * Math.abs(yControl.getAsDouble());
        }
        else if(visionSys.targetIsXAligned() || !visionSys.hasTarget()) {
            strafe = 0.0;
        }
        else {
            strafe = alignController.calculate(visionSys.targetXDegrees() - (swerveSys.getHeading().getDegrees() - 180));
        }

        if(Math.abs(strafe) > VisionConstants.maxAlignSpeedMetersPerSecond) {
            strafe = Math.copySign(VisionConstants.maxAlignSpeedMetersPerSecond, strafe);
        }

        swerveSys.drive(
            -xControl.getAsDouble() * Math.abs(xControl.getAsDouble()),
            -strafe,
            rot,
            true
        );
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSys.stop();
        swerveSys.setIsTracking(false);
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