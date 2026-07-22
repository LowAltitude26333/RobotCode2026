package org.firstinspires.ftc.teamcode.vision;

/**
 * Immutable diagnostic snapshot of the values reported by the Limelight.
 *
 * <p>This type intentionally retains non-finite values so commissioning can
 * expose a sensor or transport fault. It is diagnostic-only: actuator and pose
 * consumers must use {@link LimelightObservation}, whose numeric channel is
 * finite and fail-closed.</p>
 */
public final class LimelightRawSample {

    public final boolean devicePresent;
    public final boolean deviceConnected;
    public final boolean resultValid;
    public final long pollTimestampNanos;
    public final long controlHubTimestampMillis;
    public final int pipelineIndex;
    public final int fiducialCount;
    public final int tagId;
    public final String tagFamily;
    public final double txDegrees;
    public final double tyDegrees;
    public final double targetAreaPercent;
    public final double stalenessMs;
    public final double totalLatencyMs;
    public final boolean hasBotPose;
    public final double botPoseXMeters;
    public final double botPoseYMeters;
    public final double botPoseHeadingRadians;

    public LimelightRawSample(boolean devicePresent, boolean deviceConnected,
                              boolean resultValid, long pollTimestampNanos,
                              long controlHubTimestampMillis, int pipelineIndex,
                              int fiducialCount, int tagId, String tagFamily,
                              double txDegrees, double tyDegrees, double targetAreaPercent,
                              double stalenessMs, double totalLatencyMs,
                              boolean hasBotPose, double botPoseXMeters,
                              double botPoseYMeters, double botPoseHeadingRadians) {
        this.devicePresent = devicePresent;
        this.deviceConnected = deviceConnected;
        this.resultValid = resultValid;
        this.pollTimestampNanos = pollTimestampNanos;
        this.controlHubTimestampMillis = controlHubTimestampMillis;
        this.pipelineIndex = pipelineIndex;
        this.fiducialCount = fiducialCount;
        this.tagId = tagId;
        this.tagFamily = tagFamily == null ? "" : tagFamily;
        this.txDegrees = txDegrees;
        this.tyDegrees = tyDegrees;
        this.targetAreaPercent = targetAreaPercent;
        this.stalenessMs = stalenessMs;
        this.totalLatencyMs = totalLatencyMs;
        this.hasBotPose = hasBotPose;
        this.botPoseXMeters = botPoseXMeters;
        this.botPoseYMeters = botPoseYMeters;
        this.botPoseHeadingRadians = botPoseHeadingRadians;
    }

    public static LimelightRawSample unavailable(boolean devicePresent,
                                                  boolean deviceConnected,
                                                  long pollTimestampNanos) {
        return new LimelightRawSample(devicePresent, deviceConnected, false,
                pollTimestampNanos, 0L, -1, 0, -1, "",
                0.0, 0.0, 0.0, 0.0, 0.0,
                false, 0.0, 0.0, 0.0);
    }
}
