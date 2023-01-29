package frc.robot.commands.auto.paths;

import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSys;

public class TestTrajectory extends SwerveControllerCommand {

    private SwerveSys driveSys;

    public TestTrajectory(SwerveSys driveSys) {
        super(
            PathPlannerTrajectories.testTrajectory,
            driveSys::getPose, 
            DriveConstants.kinematics, 
            AutoConstants.driveController,
            AutoConstants.driveController,
            AutoConstants.rotationController,
            driveSys::setModuleStates, 
            driveSys
        );

        this.driveSys = driveSys;
    }

    @Override
    public void initialize() {

        super.initialize();

        driveSys.setPose(PathPlannerTrajectories.testTrajectory.getInitialPose());

    } 

    @Override
    public void execute() {

        super.execute();

    }
    
}
