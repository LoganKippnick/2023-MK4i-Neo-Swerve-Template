package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerType;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.auto.programs.RightConeGrabCube;
import frc.robot.commands.claw.CloseCmd;
import frc.robot.commands.claw.OpenCmd;
import frc.robot.commands.drivetrain.DefaultSpeedCmd;
import frc.robot.commands.drivetrain.ResetHeadingCmd;
import frc.robot.commands.drivetrain.SetLockedCmd;
import frc.robot.commands.drivetrain.SwerveDriveCmd;
import frc.robot.commands.drivetrain.TurtleSpeedCmd;
import frc.robot.commands.intake.InCmd;
import frc.robot.commands.intake.OutCmd;
import frc.robot.commands.intake.IntakeManualControlCmd;
import frc.robot.commands.lift.DownCmd;
import frc.robot.commands.lift.LiftManualControlCmd;
import frc.robot.commands.lift.Row1Cmd;
import frc.robot.commands.lift.Row2PoleCmd;
import frc.robot.commands.lift.Row2ShelfCmd;
import frc.robot.commands.lift.Row3PoleCmd;
import frc.robot.commands.lift.Row3ShelfCmd;
import frc.robot.commands.vision.RestartLimelightCmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.CompressorSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.SwerveSys;
import frc.robot.subsystems.VisionSys;

public class RobotContainer {
    
    // Initialize subsystems.
    private final SwerveSys swerveSys = new SwerveSys();
    private final LiftSys liftSys = new LiftSys();
    private final ClawSys clawSys = new ClawSys();
    private final IntakeSys intakeSys = new IntakeSys();
    private final VisionSys visionSys = new VisionSys();  
    private final CompressorSys compressorSys = new CompressorSys();  

    // Initialize joysticks.
    private final XboxController driverController = new XboxController(ControllerConstants.driverGamepadPort);

    private final Joystick driverLeftJoystick = new Joystick(ControllerConstants.driverLeftJoystickPort);
    private final Joystick driverRightJoystick = new Joystick(ControllerConstants.driverRightJoystickPort);

    private final XboxController operatorController = new XboxController(ControllerConstants.operatorGamepadPort);

    private final XboxController hybridController = new XboxController(ControllerConstants.hybridControllerPort);

    // Initialize controller buttons.
    private final JoystickButton driverLeftBumper = new JoystickButton(driverController, 5);
    private final JoystickButton driverRightBumper = new JoystickButton(driverController, 6);
    private final JoystickButton driverMenuBtn = new JoystickButton(driverController, 8);
    private final Trigger driverRightTriggerBtn =
        new Trigger(() -> driverController.getRightTriggerAxis() > ControllerConstants.triggerPressedDeadband);

    private final JoystickButton operatorABtn = new JoystickButton(operatorController, 1);
    private final JoystickButton operatorBBtn = new JoystickButton(operatorController, 2);
    private final JoystickButton operatorXBtn = new JoystickButton(operatorController, 3);
    private final JoystickButton operatorYBtn = new JoystickButton(operatorController, 4);
    private final JoystickButton operatorLeftBumper = new JoystickButton(operatorController, 5);
    private final JoystickButton operatorRightBumper = new JoystickButton(operatorController, 6);
    private final JoystickButton operatorWindowBtn = new JoystickButton(operatorController, 7);
    private final POVButton operatorUpBtn = new POVButton(operatorController, 0);
    private final POVButton operatorRightBtn = new POVButton(operatorController, 90);
    private final POVButton operatorDownBtn = new POVButton(operatorController, 180);
    private final POVButton operatorLeftBtn = new POVButton(operatorController, 270);

    private final JoystickButton hybridABtn = new JoystickButton(hybridController, 1);
    private final JoystickButton hybridBBtn = new JoystickButton(hybridController, 2);
    private final JoystickButton hybridXBtn = new JoystickButton(hybridController, 3);
    private final JoystickButton hybridYBtn = new JoystickButton(hybridController, 4);
    private final JoystickButton hybridLeftBumper = new JoystickButton(hybridController, 5);
    private final JoystickButton hybridRightBumper = new JoystickButton(hybridController, 6);
    private final JoystickButton hybridWindowBtn = new JoystickButton(hybridController, 7);
    private final JoystickButton hybridMenuBtn = new JoystickButton(driverController, 8);

    // Instantiate controller rumble.
    private Rumble matchTimeRumble;
    private Rumble brownOutRumble;
    private Rumble countdown10Rumble;
    private Rumble countdown5Rumble;

    // Initialize auto selector.
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    public RobotContainer() {
        SmartDashboard.putData(autoSelector);

        RestartLimelightCmd restartLimelight = new RestartLimelightCmd(visionSys);
        restartLimelight.setName("Restart Limelight");
        SmartDashboard.putData(restartLimelight); // TODO: See if this works.

        RunCommand toggleCompressor = new RunCommand(() -> compressorSys.setEnabled(!compressorSys.isEnabled()));
        toggleCompressor.setName("Toggle Compressor");
        SmartDashboard.putData(toggleCompressor); // TODO: See if this works.

        RobotController.setBrownoutVoltage(7.5);
    }

    public void configBindings() {
        if(DriverStation.isJoystickConnected(ControllerConstants.hybridControllerPort)) {
            configHybridBindings();
            SmartDashboard.putString("control type", "hybrid");
        }
        else {
            if(DriverStation.getJoystickIsXbox(ControllerConstants.driverGamepadPort)) {
                configDriverBindings(ControllerType.kGamepad);
                SmartDashboard.putString("control type", "gamepad");
            }
            else if(DriverStation.isJoystickConnected(ControllerConstants.driverRightJoystickPort)) {
                configDriverBindings(ControllerType.kJoystick);
                SmartDashboard.putString("control type", "joysticks");
            }
            else {
                SmartDashboard.putString("control type", "only operator");
            }

            configOperatorBindings();
        }

        brownOutRumble.rumbleWhen(() -> RobotController.isBrownedOut(), 2.0);
        
        matchTimeRumble.pulseWhen(() -> DriverStation.getMatchTime() <= 60.0 && DriverStation.isTeleop(), 3);
        matchTimeRumble.pulseWhen(() -> DriverStation.getMatchTime() <= 30.0 && DriverStation.isTeleop(), 2);
        matchTimeRumble.pulseWhen(() -> DriverStation.getMatchTime() <= 15.0 && DriverStation.isTeleop(), 1);

        countdown10Rumble.setPulseTime(1.0);
        countdown10Rumble.pulseWhen(() -> DriverStation.getMatchTime() <= 10.0 && DriverStation.isTeleop(), 5);

        countdown5Rumble.setPulseTime(1.0);
        countdown5Rumble.setPulseLength(0.25);
        countdown5Rumble.pulseWhen(() -> DriverStation.getMatchTime() <= 5.0 && DriverStation.isTeleop(), 5);
    }

    public void configDriverBindings(ControllerType driverControllerType) {
        if(driverControllerType.equals(ControllerType.kGamepad)) {
            swerveSys.setDefaultCommand(
                new SwerveDriveCmd(
                    () -> deadband(driverController.getLeftY(), driverControllerType),
                    () -> deadband(driverController.getLeftX(), driverControllerType),
                    () -> deadband(driverController.getRightX(), driverControllerType),
                    true,
                    swerveSys
                )
            );
            
            driverLeftBumper.onTrue(new SetLockedCmd(true, swerveSys)).onFalse(new SetLockedCmd(false, swerveSys));
            driverRightBumper.onTrue(new TurtleSpeedCmd(swerveSys)).onFalse(new DefaultSpeedCmd(swerveSys));
            driverMenuBtn.onTrue(new ResetHeadingCmd(swerveSys));

            brownOutRumble = new Rumble(RumbleType.kLeftRumble, 1.0, driverController);
            matchTimeRumble = new Rumble(RumbleType.kRightRumble, 1.0, driverController);
            countdown10Rumble = new Rumble(RumbleType.kRightRumble, 1.0, driverController);
            countdown5Rumble = new Rumble(RumbleType.kRightRumble, 1.0, driverController);
        }
        else {
            swerveSys.setDefaultCommand(
                new SwerveDriveCmd(
                    () -> deadband(driverLeftJoystick.getY(), driverControllerType),
                    () -> deadband(driverLeftJoystick.getX(), driverControllerType),
                    () -> deadband(driverRightJoystick.getX(), driverControllerType),
                    true,
                    swerveSys
                )
            );

            brownOutRumble = new Rumble(RumbleType.kLeftRumble, 1.0);
            matchTimeRumble = new Rumble(RumbleType.kRightRumble, 1.0);
            countdown10Rumble = new Rumble(RumbleType.kRightRumble, 1.0);
            countdown5Rumble = new Rumble(RumbleType.kRightRumble, 1.0);
        }
    }

    public void configOperatorBindings() {
        liftSys.setDefaultCommand(
            new LiftManualControlCmd(
                () -> deadband(operatorController.getRightY(), ControllerType.kGamepad),
                liftSys
            )
        );

        intakeSys.setDefaultCommand(
            new IntakeManualControlCmd(
                () -> deadband(operatorController.getLeftY(), ControllerType.kGamepad),
                () -> deadband(operatorController.getRightTriggerAxis(), ControllerType.kGamepad),
                () -> deadband(operatorController.getLeftTriggerAxis(), ControllerType.kGamepad),
                intakeSys
            )
        );

        operatorABtn.onTrue(new Row1Cmd(true, liftSys));
        operatorBBtn.onTrue(new Row2PoleCmd(true, liftSys));
        operatorXBtn.onTrue(new DownCmd(true, liftSys));
        operatorYBtn.onTrue(new Row3PoleCmd(true, liftSys));

        operatorWindowBtn.and(operatorXBtn).onTrue(new Row2ShelfCmd(true, liftSys));
        operatorWindowBtn.and(operatorYBtn).onTrue(new Row3ShelfCmd(true, liftSys));
        
        operatorLeftBumper.onTrue(new OpenCmd(clawSys));
        operatorRightBumper.onTrue(new CloseCmd(clawSys));

        operatorUpBtn.onTrue(new OutCmd(intakeSys));
        operatorDownBtn.onTrue(new InCmd(intakeSys));

        matchTimeRumble = new Rumble(RumbleType.kRightRumble, 1.0, operatorController);
        countdown10Rumble = new Rumble(RumbleType.kRightRumble, 1.0, operatorController);
        countdown5Rumble = new Rumble(RumbleType.kRightRumble, 1.0, operatorController);
    }

    public void configHybridBindings() {
        swerveSys.setDefaultCommand(
            new SwerveDriveCmd(
                () -> deadband(hybridController.getLeftY(), ControllerType.kGamepad),
                () -> deadband(hybridController.getLeftX(), ControllerType.kGamepad),
                () -> deadband(hybridController.getRightX(), ControllerType.kGamepad),
                true,
                swerveSys
            )
        );

        hybridABtn.onTrue(new Row1Cmd(true, liftSys));
        hybridBBtn.onTrue(new Row2PoleCmd(true, liftSys));
        hybridXBtn.onTrue(new DownCmd(true, liftSys));
        hybridYBtn.onTrue(new Row3PoleCmd(true, liftSys));

        hybridWindowBtn.and(operatorYBtn).onTrue(new Row3ShelfCmd(false, liftSys));
        hybridMenuBtn.onTrue(new ResetHeadingCmd(swerveSys));
        
        hybridLeftBumper.onTrue(new OpenCmd(clawSys));
        hybridRightBumper.onTrue(new CloseCmd(clawSys));

        brownOutRumble = new Rumble(RumbleType.kLeftRumble, 1.0, hybridController);
        matchTimeRumble = new Rumble(RumbleType.kRightRumble, 1.0, hybridController);
        countdown10Rumble = new Rumble(RumbleType.kRightRumble, 1.0, hybridController);
        countdown5Rumble = new Rumble(RumbleType.kRightRumble, 1.0, hybridController);
    }

    public Command getAutonomousCommand() {
        return new RightConeGrabCube(swerveSys, liftSys, clawSys, intakeSys);
        // return new DockCmd(DockDirection.kFromCenter, DockHeading.kLeft, swerveSys);
        // return new ResetPoseCmd(swerveSys).andThen(new TestTrajectory(swerveSys));
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
                ControllerConstants.gamepadDeadband :
                ControllerConstants.joystickDeadband
            )
        )
            return 0.0;
        
        return value;
    }
}
