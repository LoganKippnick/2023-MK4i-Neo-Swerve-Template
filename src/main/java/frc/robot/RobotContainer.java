package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.Controllers;
import frc.robot.commands.auto.SplineTrajectory;
import frc.robot.commands.claw.CloseCmd;
import frc.robot.commands.claw.OpenCmd;
import frc.robot.commands.drivetrain.ResetPoseCmd;
import frc.robot.commands.drivetrain.SwerveDriveCmd;
import frc.robot.commands.lift.DownCmd;
import frc.robot.commands.lift.ManualControlCmd;
import frc.robot.commands.lift.Row1Cmd;
import frc.robot.commands.lift.Row2Cmd;
import frc.robot.commands.lift.Row3Cmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.CompressorSys;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.SwerveSys;

public class RobotContainer {
    
    // Initialize subsystems.
    private final SwerveSys swerveSys = new SwerveSys();
    private final LiftSys liftSys = new LiftSys();
    private final ClawSys clawSys = new ClawSys();

    // Initialize joysticks.
    private final XboxController driverController = new XboxController(Controllers.driverControllerPort);
    private final XboxController operatorController = new XboxController(Controllers.operatorControllerPort);

    // Initialize controller buttons.
    private final JoystickButton driverMenuBtn = new JoystickButton(driverController, 8);

    private final JoystickButton operatorABtn = new JoystickButton(driverController, 1); // FIXME: revert to driverController
    private final JoystickButton operatorBBtn = new JoystickButton(driverController, 2);
    private final JoystickButton operatorXBtn = new JoystickButton(driverController, 3);
    private final JoystickButton operatorYBtn = new JoystickButton(driverController, 4);
    private final JoystickButton operatorLeftBumper = new JoystickButton(driverController, 5);
    private final JoystickButton operatorRightBumper = new JoystickButton(driverController, 6);

    // Initialize auto selector.
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    public RobotContainer() {

        new CompressorSys();

        // Set subsystem default commands, which run when no other command is scheduled.
        swerveSys.setDefaultCommand(
            new SwerveDriveCmd(
                swerveSys,
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> driverController.getRightX(),
                () -> driverController.getLeftBumper(),
                true
            )
        );

        liftSys.setDefaultCommand(
            new ManualControlCmd(
                () -> operatorController.getRightY(),
                liftSys
            )
        );

        configureButtonBindings();

        SmartDashboard.putData(autoSelector);

    }

    public void configureButtonBindings() {
        
        driverMenuBtn.onTrue(new ResetPoseCmd(swerveSys)); // FIXME: after debugging, change back to ResetHeadingCmd

        operatorABtn.onTrue(new Row1Cmd(liftSys));
        operatorBBtn.onTrue(new Row2Cmd(liftSys));
        operatorXBtn.onTrue(new DownCmd(liftSys));
        operatorYBtn.onTrue(new Row3Cmd(liftSys));
        operatorLeftBumper.onTrue(new OpenCmd(clawSys));
        operatorRightBumper.onTrue(new CloseCmd(clawSys));

    }

    public Command getAutonomousCommand() {
        return new ResetPoseCmd(swerveSys).andThen(new SplineTrajectory(swerveSys));
    }
    
}
