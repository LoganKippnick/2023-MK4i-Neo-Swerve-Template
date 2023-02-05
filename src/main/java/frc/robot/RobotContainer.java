package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Controllers;
import frc.robot.commands.auto.SplineTrajectory;
import frc.robot.commands.drivetrain.ResetPoseCmd;
import frc.robot.commands.drivetrain.SwerveDriveCmd;
import frc.robot.commands.lift.DownCmd;
import frc.robot.commands.lift.Row1Cmd;
import frc.robot.commands.lift.Row2Cmd;
import frc.robot.commands.lift.Row3Cmd;
import frc.robot.subsystems.LiftSys;
import frc.robot.subsystems.SwerveSys;

public class RobotContainer {

    /**
     * Establishes the controls and subsystems of the robot
     */

    // private final Joystick leftJoystick = new Joystick(InputDevices.leftJoystickPort);
    // private final Joystick rightJoystick = new Joystick(InputDevices.rightJoystickPort);

    private final XboxController driverController = new XboxController(Controllers.driverControllerPort);
    private final XboxController operatorController = new XboxController(Controllers.operatorControllerPort);

    private final JoystickButton driverMenuBtn = new JoystickButton(driverController, 8);

    private final JoystickButton operatorABtn = new JoystickButton(operatorController, 1);
    private final JoystickButton operatorBBtn = new JoystickButton(operatorController, 2);
    private final JoystickButton operatorXBtn = new JoystickButton(operatorController, 3);
    private final JoystickButton operatorYBtn = new JoystickButton(operatorController, 4);

    private final SwerveSys swerveSys = new SwerveSys();
    private final LiftSys liftSys = new LiftSys();
    
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

        operatorABtn.onTrue(new Row1Cmd(liftSys));
        operatorBBtn.onTrue(new Row2Cmd(liftSys));
        operatorXBtn.onTrue(new DownCmd(liftSys));
        operatorYBtn.onTrue(new Row3Cmd(liftSys));
        

    }

    public Command getAutonomousCommand() {
        return new ResetPoseCmd(swerveSys).andThen(new SplineTrajectory(swerveSys));
    }

}
