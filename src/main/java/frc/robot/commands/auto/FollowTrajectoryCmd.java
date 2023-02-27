package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSys;

public class FollowTrajectoryCmd extends PPSwerveControllerCommand {

    public FollowTrajectoryCmd(String trajectoryName, SwerveSys swerveSys) {

        super(
            PathPlanner.loadPath(trajectoryName, new PathConstraints(4.0, 3.0)),
            swerveSys::getPose,
            DriveConstants.kinematics,
            AutoConstants.driveController,
            AutoConstants.driveController,
            AutoConstants.rotController,
            swerveSys::setModuleStates,
            swerveSys
        );
    }

    public Pose2d getInitialPose() {   
        return getInitialPose();
    }
}