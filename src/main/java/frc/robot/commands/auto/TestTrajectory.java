package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSys;

public class TestTrajectory extends PPSwerveControllerCommand {

    private SwerveSys swerveSys;

    public TestTrajectory(SwerveSys swerveSys) {
        super(
            PathPlanner.loadPath("TestTrajectory", new PathConstraints(4.0, 3.0)), 
            swerveSys::getPose, 
            AutoConstants.driveController, 
            AutoConstants.driveController,
            AutoConstants.rotController, 
            swerveSys::setChassisSpeeds, 
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
