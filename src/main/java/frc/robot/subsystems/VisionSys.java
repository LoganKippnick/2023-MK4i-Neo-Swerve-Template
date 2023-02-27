package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The type of target tracked based on the pipeline.
 */
enum TargetType { // TODO: Make sure pipeline indexes are correct
    kPoleTape(0, "reflective tape"),
    kAprilTag(1, "april tag"),
    kCone(2, "cone"),
    kCube(3, "cube"),
    kNone(-1, "none");

    public final int pipelineIndex;
    public final String name;

    private TargetType(int pipelineIndex, String name) {
        this.pipelineIndex = pipelineIndex;
        this.name = name;
    }
}

public class VisionSys extends SubsystemBase {

    PhotonCamera limelight;
    
    /**
     * Constructs a new VisionSys.
     * 
     * <p>VisionSys contains the Limelight and means to control and obtain values from it.
     */
    public VisionSys() {
        limelight = new PhotonCamera("Limelight");
        setTargetType(TargetType.kNone);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        SmartDashboard.putNumber("vision pipeline", limelight.getPipelineIndex());
        SmartDashboard.putBoolean("limelight connected", limelight.isConnected());
    }

    /**
     * Returns the type of target currently being tracked.
     * @return The type of target currently being tracked.
     */
    public TargetType getTargetType() {
        if(isDriverMode()) return TargetType.kNone;
        else if(limelight.getPipelineIndex() == TargetType.kPoleTape.pipelineIndex) return TargetType.kPoleTape;
        else if(limelight.getPipelineIndex() == TargetType.kAprilTag.pipelineIndex) return TargetType.kAprilTag;
        else if(limelight.getPipelineIndex() == TargetType.kCone.pipelineIndex) return TargetType.kCone;
        else if(limelight.getPipelineIndex() == TargetType.kCone.pipelineIndex) return TargetType.kCube;
        else return TargetType.kNone;
    }

    /**
     * Sets the type of target to track.
     * @param targetType The type of target to track. If TargetType.kNone, enables driver mode, otherwise disables it.
     */
    public void setTargetType(TargetType targetType) {
        if(targetType.equals(TargetType.kNone)) {
            limelight.setDriverMode(true);
        }
        else {
            limelight.setPipelineIndex(targetType.pipelineIndex);
            limelight.setDriverMode(false);
        }
    }

    /**
     * Returns the x-offset, or yaw, from the crosshair of the best target.
     * @return The x-offset, or yaw, from the crosshair of the best target, in degrees.
     */
    public double targetXDegrees() {
        return limelight.getLatestResult().getBestTarget().getYaw();
    }

    /**
     * Returns the y-offset, or skew, from the crosshair of the best target.
     * @return The y-offset, or skew, from the crosshair of the best target, in degrees.
     */
    public double targetYDegrees() {
        return limelight.getLatestResult().getBestTarget().getPitch();
    }

    /**
     * Returns the Apriltag ID of the current target.
     * @return The Apriltag ID of the current target, -1 if the target is not an Apriltag.
     */
    public double aprilTagId() {
        return limelight.getLatestResult().getBestTarget().getFiducialId();
    }

    /**
     * Returns if driver mode is enabled.
     * @return True if driver mode is enabled.
     */
    public boolean isDriverMode() {
        return limelight.getDriverMode();
    }

    /**
     * Sets whether driver mode is enabled.
     * @param isDriverMode True if driver mode should be enabled.
     */
    public void setDriverMode(boolean isDriverMode) {
        limelight.setDriverMode(isDriverMode);
    }
}