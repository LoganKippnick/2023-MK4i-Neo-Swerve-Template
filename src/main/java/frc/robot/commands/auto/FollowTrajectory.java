package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSys;

public class FollowTrajectory extends SwerveControllerCommand {

    private final Trajectory trajectory;

    public FollowTrajectory(SwerveSys swerveSys, Trajectory trajectory) {

        super(
            trajectory, 
            swerveSys::getPose, 
            DriveConstants.kinematics, 
            new PIDController(2.45, 0, 1.0),
            new PIDController(2.45, 0, 1.0), 
            AutoConstants.rotController,
            swerveSys::setModuleStates, 
            swerveSys
        );

        this.trajectory = trajectory;
    
    }

    public Pose2d getInitialPose() {
        
        return trajectory.getInitialPose();

    }

}