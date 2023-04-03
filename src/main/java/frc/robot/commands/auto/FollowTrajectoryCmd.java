package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSys;

public class FollowTrajectoryCmd extends PPSwerveControllerCommand {

    private final SwerveSys swerveSys;

    public FollowTrajectoryCmd(String trajectoryName, double maxVelMetersPerSec, double maxAccelMetersPerSecondSq, SwerveSys swerveSys) {
        super(
            PathPlanner.loadPath(trajectoryName, maxVelMetersPerSec, maxAccelMetersPerSecondSq),
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
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        swerveSys.stop();
    }

    public FollowTrajectoryCmd(String trajectoryName, SwerveSys swerveSys) {
        this(trajectoryName, AutoConstants.maxVelMetersPerSec, AutoConstants.maxAccelMetersPerSecondSq, swerveSys);
    }
}