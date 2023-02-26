package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSys;

public class FollowTrajectoryCmd extends SwerveControllerCommand {

    private final Trajectory trajectory;

    public FollowTrajectoryCmd(String trajectoryName, SwerveSys swerveSys) {

        super(
            createTrajectory(trajectoryName),
            swerveSys::getPose,
            DriveConstants.kinematics,
            AutoConstants.driveController,
            AutoConstants.driveController,
            AutoConstants.rotController,
            swerveSys::setModuleStates,
            swerveSys
        );

        this.trajectory = createTrajectory(trajectoryName);
    
    }

    public Pose2d getInitialPose() {   
        return trajectory.getInitialPose();
    }

    /**
     * Creates trajectories from a JSON file in deploy directory.
     * 
     * @param fileName The name of the JSON file to create a trajectory for (excluding the file extension).
     * 
     * @return The trajectory created from the JSON file.
     */
    public static Trajectory createTrajectory(String trajectoryName) {
        Trajectory trajectory;
        Path trajectoryPath =
            Filesystem.getDeployDirectory().toPath().resolve(AutoConstants.trajectoryFileLocation + trajectoryName + ".wpilib.json");

        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            return trajectory;
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryName, ex.getStackTrace());
            return new Trajectory();
        }
    }

}