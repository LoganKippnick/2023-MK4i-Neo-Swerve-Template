package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSys;

public class FollowTrajectoryCmd extends PPSwerveControllerCommand {

    private final SwerveSys swerveSys;
    
    private final String trajectoryName;
    private final double maxVelMetersPerSec;
    private final double maxAccelMetersPerSecondSq;

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
        this.trajectoryName = trajectoryName;
        this.maxVelMetersPerSec = maxVelMetersPerSec;
        this.maxAccelMetersPerSecondSq = maxAccelMetersPerSecondSq;
    }

    @Override
    public void initialize() {
        swerveSys.setField2dTrajectory(PathPlanner.loadPath(trajectoryName, maxVelMetersPerSec, maxAccelMetersPerSecondSq));
    }

    public FollowTrajectoryCmd(String trajectoryName, SwerveSys swerveSys) {
        this(trajectoryName, AutoConstants.maxVelMetersPerSec, AutoConstants.maxAccelMetersPerSecondSq, swerveSys);
    }
}