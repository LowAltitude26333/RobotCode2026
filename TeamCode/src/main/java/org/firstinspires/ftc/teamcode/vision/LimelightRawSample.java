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
    public final boolean resultValid;
    public final long timestampNanos;
    public final int tagId;
    public final double txDegrees;
    public final double tyDegrees;
    public final double stalenessMs;
    public final double totalLatencyMs;
    public final boolean hasBotPose;
    public final double botPoseXMeters;
    public final double botPoseYMeters;
    public final double botPoseHeadingRadians;

    public LimelightRawSample(boolean devicePresent, boolean resultValid,
                              long timestampNanos, int tagId,
                              double txDegrees, double tyDegrees,
                              double stalenessMs, double totalLatencyMs,
                              boolean hasBotPose, double botPoseXMeters,
                              double botPoseYMeters, double botPoseHeadingRadians) {
        this.devicePresent = devicePresent;
        this.resultValid = resultValid;
        this.timestampNanos = timestampNanos;
        this.tagId = tagId;
        this.txDegrees = txDegrees;
        this.tyDegrees = tyDegrees;
        this.stalenessMs = stalenessMs;
        this.totalLatencyMs = totalLatencyMs;
        this.hasBotPose = hasBotPose;
        this.botPoseXMeters = botPoseXMeters;
        this.botPoseYMeters = botPoseYMeters;
        this.botPoseHeadingRadians = botPoseHeadingRadians;
    }

    public static LimelightRawSample unavailable(boolean devicePresent, long timestampNanos) {
        return new LimelightRawSample(devicePresent, false, timestampNanos, -1,
                0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0);
    }
}
