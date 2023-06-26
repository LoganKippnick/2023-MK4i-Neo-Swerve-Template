package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.drivetrain.LockCmd;
import frc.robot.commands.drivetrain.ResetHeadingCmd;
import frc.robot.commands.drivetrain.SwerveDriveCmd;
import frc.robot.commands.intake.InCmd;
import frc.robot.commands.intake.IntakeConeCmd;
import frc.robot.commands.intake.OutCmd;
import frc.robot.commands.intake.IntakeCubeCmd;
import frc.robot.commands.intake.StopRollersCmd;
import frc.robot.commands.intake.IntakeManualControlCmd;
import frc.robot.commands.lift.DownCmd;
import frc.robot.commands.lift.HoverCmd;
import frc.robot.commands.lift.LiftManualControlCmd;
import frc.robot.commands.lift.Row1Cmd;
import frc.robot.commands.lift.Row2Cmd;
import frc.robot.commands.lift.Row3Cmd;
import frc.robot.commands.lift.ShelfPickupCmd;
import frc.robot.commands.lights.TogglePartyModeCmd;
import frc.robot.commands.lights.ToggleWeeWooModeCmd;
import frc.robot.commands.vision.RestartLimelightCmd;
import frc.robot.subsystems.ClawSys;
import frc.robot.subsystems.CompressorSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.LightsSys;
import frc.robot.subsystems.SwerveSys;

public class RobotContainer {
    
    // Initialize subsystems.
    private final SwerveSys swerveSys = new SwerveSys();

    // Initialize joysticks.
    private final XboxController driverController = new XboxController(ControllerConstants.driverGamepadPort);

    // Initialize controller buttons.
    private final JoystickButton driverRightJoystickTriggerBtn = new JoystickButton(driverRightJoystick, 1);
    private final JoystickButton driverRightJoystickThumbBtn = new JoystickButton(driverRightJoystick, 2);

    private final JoystickButton driverABtn = new JoystickButton(driverController, 1);
    // private final JoystickButton driverBBtn = new JoystickButton(driverController, 2);
    private final JoystickButton driverXBtn = new JoystickButton(driverController, 3);
    private final JoystickButton driverYBtn = new JoystickButton(driverController, 4);
    // private final JoystickButton driverLeftBumper = new JoystickButton(driverController, 5);
    private final JoystickButton driverRightBumper = new JoystickButton(driverController, 6);
    private final JoystickButton driverMenuBtn = new JoystickButton(driverController, 8);
    private final Trigger driverRightTriggerBtn =
        new Trigger(() -> driverController.getRightTriggerAxis() > ControllerConstants.triggerPressedDeadband);
    private final Trigger driverLeftTriggerBtn =
        new Trigger(() -> driverController.getLeftTriggerAxis() > ControllerConstants.triggerPressedDeadband);

    private final JoystickButton operatorABtn = new JoystickButton(operatorController, 1);
    private final JoystickButton operatorBBtn = new JoystickButton(operatorController, 2);
    private final JoystickButton operatorXBtn = new JoystickButton(operatorController, 3);
    private final JoystickButton operatorYBtn = new JoystickButton(operatorController, 4);
    private final JoystickButton operatorLeftBumper = new JoystickButton(operatorController, 5);
    private final JoystickButton operatorRightBumper = new JoystickButton(operatorController, 6);
    private final JoystickButton operatorWindowBtn = new JoystickButton(operatorController, 7);
    private final JoystickButton operatorMenuBtn = new JoystickButton(operatorController, 8);
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
    private final JoystickButton hybridMenuBtn = new JoystickButton(hybridController, 8);
    private final JoystickButton hybridRightJoystickPressBtn = new JoystickButton(hybridController, 10);
    
    private final Trigger hybridRightTriggerBtn =
        new Trigger(() -> hybridController.getRightTriggerAxis() > ControllerConstants.triggerPressedDeadband);
    private final Trigger hybridLeftTriggerBtn =
        new Trigger(() -> hybridController.getLeftTriggerAxis() > ControllerConstants.triggerPressedDeadband);

    private final POVButton hybridUpBtn = new POVButton(hybridController, 0);
    private final POVButton hybridRightBtn = new POVButton(hybridController, 90);
    private final POVButton hybridDownBtn = new POVButton(hybridController, 180);
    private final POVButton hybridLeftBtn = new POVButton(hybridController, 270);

    // Instantiate controller rumble.
    private Rumble matchTimeRumble;
    private Rumble brownOutRumble;
    private Rumble countdown10Rumble;
    private Rumble countdown5Rumble;
    private Rumble targetAlignedRumble;

    // Initialize auto selector.
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    public RobotContainer() {
        SmartDashboard.putData("auto selector", autoSelector);

        configDriverBindings();
        configOperatorBindings();
    }

    public void configBindings() {
        lightsSys.cancelAnimations();

        brownOutRumble = new Rumble(RumbleType.kLeftRumble, 1.0, driverController);
        matchTimeRumble = new Rumble(RumbleType.kRightRumble, 1.0, driverController);
        countdown10Rumble = new Rumble(RumbleType.kRightRumble, 1.0, driverController);
        countdown5Rumble = new Rumble(RumbleType.kRightRumble, 1.0, driverController);
        targetAlignedRumble = new Rumble(RumbleType.kLeftRumble, 1.0, driverController);

        if(DriverStation.isJoystickConnected(ControllerConstants.hybridControllerPort)) {
            configHybridBindings();
            // SmartDashboard.putString("control type", "hybrid");
        }
        else {
            if(DriverStation.getJoystickIsXbox(ControllerConstants.driverGamepadPort)) {
                configDriverBindings(ControllerType.kGamepad);
                // SmartDashboard.putString("control type", "gamepad");
            }
            else if(DriverStation.isJoystickConnected(ControllerConstants.driverRightJoystickPort)) {
                configDriverBindings(ControllerType.kJoystick);
                // SmartDashboard.putString("control type", "joysticks");
            }
            else {
                // SmartDashboard.putString("control type", "only operator");
                brownOutRumble = new Rumble(RumbleType.kRightRumble, 1.0);
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

        targetAlignedRumble.pulseWhen(() -> visionSys.targetIsXAligned() && visionSys.hasTarget() && swerveSys.isTracking());
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

            driverABtn.onTrue(new ToggleWeeWooModeCmd(lightsSys));
            driverXBtn.whileTrue(new AutoAlignCmd(
                () -> deadband(driverController.getLeftY(), driverControllerType),
                () -> deadband(driverController.getLeftX(), driverControllerType),
                visionSys, swerveSys, liftSys)
            );
            driverYBtn.onTrue(new TogglePartyModeCmd(lightsSys));
            
            driverMenuBtn.onTrue(new ResetHeadingCmd(swerveSys));

            driverRightTriggerBtn
                .onTrue(new OutCmd(intakeSys, lightsSys))
                .onTrue(new IntakeCubeCmd(intakeSys, lightsSys))
                .onFalse(new InCmd(intakeSys))
                .onFalse(new StopRollersCmd(intakeSys, lightsSys));

            driverRightBumper
                .onTrue(new OutCmd(intakeSys, lightsSys))
                .whileTrue(new IntakeConeCmd(intakeSys, lightsSys))
                .onFalse(new InCmd(intakeSys))
                .onFalse(new StopRollersCmd(intakeSys, lightsSys));

            driverLeftTriggerBtn.whileTrue(new LockCmd(swerveSys));

            brownOutRumble.addControllers(driverController);
            matchTimeRumble.addControllers(driverController);
            countdown10Rumble.addControllers(driverController);
            countdown5Rumble.addControllers(driverController);
            targetAlignedRumble.addControllers(driverController);
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

            driverRightJoystickTriggerBtn
                .onTrue(new OutCmd(intakeSys, lightsSys))
                .whileTrue(new IntakeCubeCmd(intakeSys, lightsSys))
                .onFalse(new InCmd(intakeSys))
                .onFalse(new StopRollersCmd(intakeSys, lightsSys));

            driverRightJoystickThumbBtn.onTrue(new ResetHeadingCmd(swerveSys));
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
        operatorBBtn.onTrue(new Row2Cmd(lightsSys, true, liftSys));
        operatorXBtn.onTrue(new DownCmd(true, liftSys));
        operatorYBtn.onTrue(new Row3Cmd(lightsSys, true, liftSys));

        operatorWindowBtn.onTrue(new SetElementStatusCmd(GameElement.kCube, liftSys, intakeSys, visionSys, lightsSys));
        operatorMenuBtn.onTrue(new SetElementStatusCmd(GameElement.kCone, liftSys, intakeSys, visionSys, lightsSys));

        operatorWindowBtn.and(operatorMenuBtn).onTrue(new SetElementStatusCmd(GameElement.kNone, liftSys, intakeSys, visionSys, lightsSys));
        
        operatorLeftBumper.onTrue(new OpenCmd(clawSys));
        operatorRightBumper.onTrue(new CloseCmd(clawSys));

        operatorUpBtn.onTrue(new ShelfPickupCmd(true, liftSys));
        operatorRightBtn.onTrue(new YEETCmd(liftSys, clawSys));
        operatorDownBtn.onTrue(new HoverCmd(true, liftSys));
        operatorLeftBtn.onTrue(new HybridYeetCmd(liftSys, clawSys));

        matchTimeRumble.addControllers(operatorController);
        countdown10Rumble.addControllers(operatorController);
        countdown5Rumble.addControllers(operatorController);
        targetAlignedRumble.addControllers(operatorController);
    }

    public void configHybridBindings() {
        swerveSys.setDefaultCommand(
            new SwerveDriveCmd(
                () -> deadband(driverController.getLeftY()),
                () -> deadband(driverController.getLeftX()),
                () -> deadband(driverController.getRightX()),
                true,
                swerveSys
            )
        );

        driverRightBumper.whileTrue(new LockCmd(swerveSys));

        driverMenuBtn.onTrue(new ResetHeadingCmd(swerveSys));
    }

        hybridWindowBtn.and(hybridMenuBtn).onTrue(new SetElementStatusCmd(GameElement.kNone, liftSys, intakeSys, visionSys, lightsSys));
        
        hybridLeftBumper.onTrue(new OpenCmd(clawSys));
        hybridRightBumper.onTrue(new CloseCmd(clawSys));

        hybridRightJoystickPressBtn.onTrue(new ResetHeadingCmd(swerveSys));

        hybridRightTriggerBtn
                .onTrue(new OutCmd(intakeSys, lightsSys))
                .whileTrue(new RepeatCommand(new IntakeCubeCmd(intakeSys, lightsSys)))
                .onFalse(new InCmd(intakeSys))
                .onFalse(new StopRollersCmd(intakeSys, lightsSys));
        hybridLeftTriggerBtn.whileTrue(new LockCmd(swerveSys));

        hybridUpBtn.onTrue(new ShelfPickupCmd(true, liftSys));
        hybridRightBtn.onTrue(new YEETCmd(liftSys, clawSys));
        hybridDownBtn.onTrue(new HoverCmd(true, liftSys));
        hybridLeftBtn.onTrue(new HybridYeetCmd(liftSys, clawSys));

        brownOutRumble.addControllers(hybridController);
        matchTimeRumble.addControllers(hybridController);
        countdown10Rumble.addControllers(hybridController);
        countdown5Rumble.addControllers(hybridController);
        targetAlignedRumble.addControllers(hybridController);
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    }

    /**
     * Deadbands inputs to eliminate tiny unwanted values from the joysticks or gamepad sticks.
     * <p>If the distance between the input and zero is less than the deadband amount, the output will be zero.
     * Otherwise, the value will not change.
     * 
     * @param input The controller value to deadband.
     * have different deadbands.
     * @return The deadbanded controller value.
     */
    public double deadband(double value) {

        if (Math.abs(value) < ControllerConstants.joystickDeadband)
            return 0.0;
        
        return value;
    }

    public void updateInterface() {
        // BATTERY
        SmartDashboard.putNumber("battery voltage", RobotController.getBatteryVoltage());

        // SWERVE
        double headingDisplay = swerveSys.getHeading().getDegrees() % 360;
        if(headingDisplay < 0) {
            headingDisplay += 360;
        }

        SmartDashboard.putNumber("heading", headingDisplay);

        SmartDashboard.putNumber("speed m/s", swerveSys.getAverageDriveVelocityMetersPerSecond());
        SmartDashboard.putNumber("speed mph", swerveSys.getAverageDriveVelocityMetersPerSecond() * 2.23694);
    }
}
