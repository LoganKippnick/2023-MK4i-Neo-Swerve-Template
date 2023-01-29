package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Controllers;
import frc.robot.commands.auto.paths.SplineTrajectory;
import frc.robot.commands.drivetrain.ResetPoseCmd;
import frc.robot.commands.drivetrain.SwerveDriveCmd;
import frc.robot.subsystems.SwerveSys;

public class RobotContainer {

    /**
     * Establishes the controls and subsystems of the robot
     */

    // private final Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
    // private final Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);

    private final XboxController gamepad = new XboxController(Controllers.operatorControllerPort);

    private final JoystickButton gamepadMenuBtn = new JoystickButton(gamepad, 8);

    private final SwerveSys driveSys = new SwerveSys();
    
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    public RobotContainer() {

        driveSys.setDefaultCommand(
            new SwerveDriveCmd(
                driveSys,
                () -> gamepad.getRawAxis(1),
                () -> gamepad.getRawAxis(0),
                () -> gamepad.getRawAxis(4),
                () -> gamepad.getLeftBumper(),
                true
            )
        );

        configureButtonBindings();

        SmartDashboard.putData(autoSelector);

    }

    public void configureButtonBindings() {
        
        gamepadMenuBtn.onTrue(new ResetPoseCmd(driveSys)); // FIXME: after debugging, change back to ResetHeadingCmd

    }

    public Command getAutonomousCommand() {
        return new ResetPoseCmd(driveSys).andThen(new SplineTrajectory(driveSys));
    }

}
