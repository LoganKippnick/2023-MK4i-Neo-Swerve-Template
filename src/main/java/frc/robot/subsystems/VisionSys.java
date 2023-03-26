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
    public static enum Pipeline {
        kPoleTape(0, "reflective tape"),
        kAprilTag(1, "april tag"),
        kIntake(2, "intake");

        public final int pipelineIndex;
        public final String name;

        private Pipeline(int pipelineIndex, String name) {
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
        setPipeline(Pipeline.kIntake);

        powerDistributionHub = new PowerDistribution(1, ModuleType.kRev);
        setPower(true);
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
    }

    /**
     * Returns the type of target currently being tracked.
     * @return The type of target currently being tracked.
     */
    public Pipeline getPipeline() {
        if(limelight.getPipelineIndex() == Pipeline.kPoleTape.pipelineIndex) return Pipeline.kPoleTape;
        else if(limelight.getPipelineIndex() == Pipeline.kAprilTag.pipelineIndex) return Pipeline.kAprilTag;
        else return Pipeline.kIntake;
    }

    /**
     * Sets the type of target to track.
     * @param pipeline The type of target to track. If Pipeline.kNone, enables driver mode, otherwise disables it.
     */
    public void setPipeline(Pipeline pipeline) {
        limelight.setPipelineIndex(pipeline.pipelineIndex);
        if(pipeline.equals(Pipeline.kIntake)) {
            limelight.setDriverMode(true);
        }
        else {
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
    }

    /**
     * Sets whether the limelight should be powered.
     * @param isPowered Whether the switchable channel should enable.
     */
    public void setPower(boolean isPowered) {
        powerDistributionHub.setSwitchableChannel(isPowered);
    }
}