package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSys extends SubsystemBase {

    private final PowerDistribution powerDistributionHub;

    /**
     * The type of target tracked based on the pipeline.
     */
    public static enum TargetType { // TODO: Make sure pipeline indexes are correct
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

    PhotonCamera limelight;
    
    /**
     * Constructs a new VisionSys.
     * 
     * <p>VisionSys contains the Limelight and means to control and obtain values from it.
     */
    public VisionSys() {
        limelight = new PhotonCamera("Limelight");
        setTargetType(TargetType.kNone);

        powerDistributionHub = new PowerDistribution(1, ModuleType.kRev);
        setPower(true); // TODO: Try this. If it still doesn't work, set the PDH's CAN ID to 1.
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
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
     * Checks whether the limelight is tracking a target.
     * @return True if the limelight is tracking a target.
     */
    public boolean hasTarget() {
        return limelight.getLatestResult().hasTargets();
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
     * Checks whether the target is aligned.
     * @return True if the target is within the alignment threshold.
     */
    public boolean targetIsXAligned() {
        return Math.abs(targetXDegrees()) < VisionConstants.alignedToleranceDegrees;
        // test(porpoises); -Andy
    }

    /**
     * Returns the Apriltag ID of the current target.
     * @return The Apriltag ID of the current target, -1 if the target is not an Apriltag.
     */
    public double aprilTagId() {
        return limelight.getLatestResult().getBestTarget().getFiducialId();
    }

    /**
     * Checks if driver mode is enabled.
     * @return True if driver mode is enabled.
     */
    public boolean isDriverMode() {
        return limelight.getDriverMode();
    }

    /**
     * Sets whether driver mode is enabled.
     * @param isDriverMode Whether driver mode should be enabled.
     */
    public void setDriverMode(boolean isDriverMode) {
        limelight.setDriverMode(isDriverMode);
    }

    /**
     * Checks whether the limelight is connected.
     * @return True if the limelight is connected.
     */
    public boolean isConnected() {
        return limelight.isConnected();
    }

    /**
     * Checks whether the limelight is being powered.
     * @return True if the PDH's switchable channel is enabled.
     */
    public boolean isPowered() {
        return powerDistributionHub.getSwitchableChannel();
        // FIXME
    }

    /**
     * Sets whether the limelight should be powered.
     * @param isPowered Whether the switchable channel should enable.
     */
    public void setPower(boolean isPowered) {
        powerDistributionHub.setSwitchableChannel(isPowered);
        // FIXME
    }
}