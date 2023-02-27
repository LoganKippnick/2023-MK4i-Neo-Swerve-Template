package frc.robot.commands.automation;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.SwerveSys;
import frc.robot.subsystems.VisionSys;
import frc.robot.subsystems.VisionSys.TargetType;

public class AutoAlignCmd extends CommandBase {

    private final VisionSys visionSys;
    private final SwerveSys swerveSys;
    private final LiftSys liftSys;

    private final DoubleSupplier xControl;

    private final PIDController alignController;
    private final PIDController rotController;

    private final TargetType targetType;
    
    public AutoAlignCmd(DoubleSupplier xControl, TargetType targetType, VisionSys visionSys, SwerveSys swerveSys, LiftSys liftSys) {
        this.visionSys = visionSys;
        this.swerveSys = swerveSys;
        this.liftSys = liftSys;
        this.xControl = xControl;
        this.targetType = targetType;

        alignController = VisionConstants.alignController;
        alignController.setSetpoint(0.0);

        rotController = AutoConstants.rotController;
        rotController.setSetpoint(Math.PI);
        
        addRequirements(visionSys, swerveSys);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        visionSys.setTargetType(targetType);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveSys.drive(
            -xControl.getAsDouble(),
            alignController.calculate(visionSys.targetXDegrees() - swerveSys.getHeading().getDegrees() - 180),
            rotController.calculate(swerveSys.getHeading().getRadians()),
            true
        );
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSys.stop();
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return xControl.getAsDouble() < 0 && Math.abs(swerveSys.getAverageDriveDistanceMeters()) < 0.01 && visionSys.targetIsXAligned(); // FIXME: add lift conditions
    }

    // Whether the command should run when robot is disabled.
    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}