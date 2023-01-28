package frc.robot.commands.auto.paths;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class PathPlannerTrajectories {

    private static final Path testTrajectoryPath =
        Filesystem.getDeployDirectory().toPath().resolve("pathplanner/generatedJSON/TestTrajecteory.wpilib.json");
    public static Trajectory testTrajectory;

    public static void createTrajectories() {
        try {
            testTrajectory = TrajectoryUtil.fromPathweaverJson(testTrajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + testTrajectory, ex.getStackTrace());
        }
    }
    
}
