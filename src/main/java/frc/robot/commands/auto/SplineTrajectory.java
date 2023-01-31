package frc.robot.commands.auto;

import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSys;

public class SplineTrajectory extends SwerveControllerCommand {

    public SplineTrajectory(SwerveSys swerveSys) {

        super(
            TrajectoryGenerator.generateTrajectory(
                swerveSys.getPose(),
                List.of(
                    new Translation2d(1.0, 0.5),
                    new Translation2d(2.0, -0.5)                    
                ), 
                new Pose2d(3.0, 0.0, new Rotation2d(0)),
                AutoConstants.config
            ),
            swerveSys::getPose, 
            DriveConstants.kinematics, 
            AutoConstants.driveController,
            AutoConstants.driveController,
            AutoConstants.rotController,
            swerveSys::setModuleStates, 
            swerveSys
        );
    }

    @Override
    public void initialize() {

        super.initialize();

    } 

    @Override
    public void execute() {

        super.execute();

    }
    
}
