package frc.robot.commands.auto.paths;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSys;

public class NewPath extends SequentialCommandGroup {
  public NewPath(DriveSys driveSys) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("New Path", 1, 3, false);

    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
        trajectory,
        driveSys::getPose,
        DriveConstants.kinematics,
        new PIDController(DriveConstants.drivekP, 0, 0),
        new PIDController(DriveConstants.drivekP, 0, 0),
        new PIDController(DriveConstants.rotationkP, 0, DriveConstants.rotationkD),
        driveSys::setModuleStates,
        true,
        driveSys);

    addCommands(
        new RunCommand(() -> driveSys.resetPose(new Pose2d())),
        command,
        new RunCommand(() -> driveSys.drive(0, 0, 0, false, false))
    );
  }
}
