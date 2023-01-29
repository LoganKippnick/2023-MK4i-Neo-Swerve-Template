package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Controllers;
import frc.robot.commands.auto.SplineTrajectory;
import frc.robot.commands.drivetrain.ResetPoseCmd;
import frc.robot.commands.drivetrain.SwerveDriveCmd;
import frc.robot.subsystems.SwerveSys;

public class RobotContainer {

    /**
     * Establishes the controls and subsystems of the robot
     */

    // private final Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
    // private final Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);

    private final XboxController driverController = new XboxController(Controllers.driverControllerPort);

    private final JoystickButton driverMenuBtn = new JoystickButton(driverController, 8);

    private final SwerveSys swerveSys = new SwerveSys();
    
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();

    public RobotContainer() {

        swerveSys.setDefaultCommand(
            new SwerveDriveCmd(
                swerveSys,
                () -> driverController.getRawAxis(1),
                () -> driverController.getRawAxis(0),
                () -> driverController.getRawAxis(4),
                () -> driverController.getLeftBumper(),
                true
            )
        );

        configureButtonBindings();

        SmartDashboard.putData(autoSelector);

    }

    public void configureButtonBindings() {
        
        driverMenuBtn.onTrue(new ResetPoseCmd(swerveSys)); // FIXME: after debugging, change back to ResetHeadingCmd

    }

    public Command getAutonomousCommand() {
        return new ResetPoseCmd(swerveSys).andThen(new SplineTrajectory(swerveSys));
    }

}
