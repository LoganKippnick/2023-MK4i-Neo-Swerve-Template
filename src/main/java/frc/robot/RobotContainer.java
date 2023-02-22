package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerType;
import frc.robot.Constants.Controllers;
import frc.robot.commands.auto.SplineTrajectory;
import frc.robot.commands.claw.CloseCmd;
import frc.robot.commands.claw.OpenCmd;
import frc.robot.commands.drivetrain.DefaultSpeedCmd;
import frc.robot.commands.drivetrain.ResetPoseCmd;
import frc.robot.commands.drivetrain.SetLockedCmd;
import frc.robot.commands.drivetrain.SwerveDriveCmd;
import frc.robot.commands.drivetrain.TurtleSpeedCmd;
import frc.robot.commands.lift.DownCmd;
import frc.robot.commands.lift.ManualControlCmd;
import frc.robot.commands.lift.Row1Cmd;
import frc.robot.commands.lift.Row2Cmd;
import frc.robot.commands.lift.Row3PoleCmd;
import frc.robot.commands.lift.Row3ShelfCmd;
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
    private final XboxController driverController = new XboxController(Controllers.driverGamepadPort);
    private final XboxController operatorController = new XboxController(Controllers.operatorGamepadPort);

    private ControllerType driverControllerType;

    // Initialize controller buttons.
    private final JoystickButton driverLeftBumper = new JoystickButton(driverController, 5);
    private final JoystickButton driverRightBumper = new JoystickButton(driverController, 6);
    private final JoystickButton driverMenuBtn = new JoystickButton(driverController, 8);

    private final JoystickButton operatorABtn = new JoystickButton(operatorController, 1);
    private final JoystickButton operatorBBtn = new JoystickButton(operatorController, 2);
    private final JoystickButton operatorXBtn = new JoystickButton(operatorController, 3);
    private final JoystickButton operatorYBtn = new JoystickButton(operatorController, 4);
    private final JoystickButton operatorLeftBumper = new JoystickButton(operatorController, 5);
    private final JoystickButton operatorRightBumper = new JoystickButton(operatorController, 6);
    private final JoystickButton operatorWindowBtn = new JoystickButton(operatorController, 7);

    // Initialize auto selector.
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    public RobotContainer() {

        new CompressorSys();

        // Set subsystem default commands, which run when no other command is scheduled.
        swerveSys.setDefaultCommand(
            new SwerveDriveCmd(
                () -> deadband(driverController.getRawAxis(1), driverControllerType),
                () -> deadband(driverController.getRawAxis(0), driverControllerType),
                () -> deadband(driverController.getRawAxis(4), driverControllerType),
                true,
                swerveSys
            )
        );

        liftSys.setDefaultCommand(
            new ManualControlCmd(
                () -> deadband(operatorController.getRightY(), ControllerType.kGamepad),
                liftSys
            )
        );

        configureButtonBindings();

        SmartDashboard.putData(autoSelector);

    }

    public void configureButtonBindings() {

        driverControllerType = ControllerType.kGamepad;
        
        driverLeftBumper.onTrue(new SetLockedCmd(true, swerveSys)).onFalse(new SetLockedCmd(false, swerveSys));
        driverRightBumper.onTrue(new TurtleSpeedCmd(swerveSys)).onFalse(new DefaultSpeedCmd(swerveSys));
        driverMenuBtn.onTrue(new ResetPoseCmd(swerveSys)); // FIXME: after debugging, change back to ResetHeadingCmd

        operatorABtn.onTrue(new Row1Cmd(liftSys));
        operatorBBtn.onTrue(new Row2Cmd(liftSys));
        operatorXBtn.onTrue(new DownCmd(liftSys));
        operatorYBtn.onTrue(new Row3PoleCmd(liftSys));

        operatorWindowBtn.and(operatorYBtn).onTrue(new Row3ShelfCmd(liftSys));
        
        operatorLeftBumper.onTrue(new OpenCmd(clawSys));
        operatorRightBumper.onTrue(new CloseCmd(clawSys));

    }

    public Command getAutonomousCommand() {
        return new ResetPoseCmd(swerveSys).andThen(new SplineTrajectory(swerveSys));
    }

    /**
     * Deadbands inputs to eliminate tiny unwanted values from the joysticks or gamepad sticks.
     * <p>If the distance between the input and zero is less than the deadband amount, the output will be zero.
     * Otherwise, the value will not change.
     * 
     * @param input The controller value to deadband.
     * @param controllerType The type of controller, since joysticks, gamepad sticks, and gamepad triggers can
     * have different deadbands.
     * @return The deadbanded controller value.
     */
    public double deadband(double value, ControllerType controllerType) {

        if (Math.abs(value) < (controllerType.equals(ControllerType.kGamepad) ?
                Controllers.gamepadDeadband :
                Controllers.joystickDeadband
            )
        )
            return 0.0;
        
        return value;

    }

}
