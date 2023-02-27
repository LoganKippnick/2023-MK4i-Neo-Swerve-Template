package frc.robot.commands.auto;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.AutoConstants;

public class PathPlannerTrajectories {

    public static final Trajectory testTrajectory = createTrajectory("TestTrajectory");

    /**
     * Creates trajectories from a JSON file in deploy directory.
     * 
     * @param fileName The name of the JSON file to create a trajectory for (excluding the file extension).
     * 
     * @return The trajectory created from the JSON file.
     */
    public static Trajectory createTrajectory(String fileName) {
        Trajectory trajectory;
        Path trajectoryPath =
            Filesystem.getDeployDirectory().toPath().resolve(AutoConstants.trajectoryFileLocation + fileName + ".wpilib.json");

        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            return trajectory;
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + fileName, ex.getStackTrace());
            return new Trajectory();
        }
    }
    
}
