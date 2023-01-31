package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSys;

public class TestTrajectory extends SwerveControllerCommand {

    private SwerveSys swerveSys;

    public TestTrajectory(SwerveSys swerveSys) {
        super(
            PathPlannerTrajectories.testTrajectory,
            swerveSys::getPose, 
            DriveConstants.kinematics, 
            AutoConstants.driveController,
            AutoConstants.driveController,
            AutoConstants.rotController,
            swerveSys::setModuleStates, 
            swerveSys
        );

        this.swerveSys = swerveSys;
    }

    @Override
    public void initialize() {

        super.initialize();

        swerveSys.setPose(PathPlannerTrajectories.testTrajectory.getInitialPose());

    } 

    @Override
    public void execute() {

        super.execute();

    }
    
}
