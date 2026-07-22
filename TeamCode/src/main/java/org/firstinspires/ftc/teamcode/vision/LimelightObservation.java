package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.util.Angles;

/**
 * Sanitized immutable observation for actuator and pose consumers.
 *
 * <p>Only {@link Quality#VALID} is actionable. Every rejected observation has
 * zeroed finite numeric fields; the unsanitized diagnostic values remain in
 * the separate {@link LimelightRawSample} published by the subsystem.</p>
 */
public final class LimelightObservation {

    public enum Quality {
        VALID,
        NO_TARGET,
        WRONG_TAG,
        STALE,
        DEVICE_ABSENT,
        DEVICE_DISCONNECTED,
        WRONG_PIPELINE,
        OUT_OF_BOUNDS,
        INVALID
    }

    public final Quality quality;
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
    public final String rejectionReason;

    private LimelightObservation(Quality quality, long pollTimestampNanos,
                                 long controlHubTimestampMillis, int pipelineIndex,
                                 int fiducialCount, int tagId, String tagFamily,
                                 double txDegrees, double tyDegrees, double targetAreaPercent,
                                 double stalenessMs, double totalLatencyMs,
                                 boolean hasBotPose, double botPoseXMeters,
                                 double botPoseYMeters, double botPoseHeadingRadians,
                                 String rejectionReason) {
        this.quality = quality;
        this.pollTimestampNanos = pollTimestampNanos;
        this.controlHubTimestampMillis = controlHubTimestampMillis;
        this.pipelineIndex = pipelineIndex;
        this.fiducialCount = fiducialCount;
        this.tagId = tagId;
        this.tagFamily = tagFamily;
        this.txDegrees = txDegrees;
        this.tyDegrees = tyDegrees;
        this.targetAreaPercent = targetAreaPercent;
        this.stalenessMs = stalenessMs;
        this.totalLatencyMs = totalLatencyMs;
        this.hasBotPose = hasBotPose;
        this.botPoseXMeters = botPoseXMeters;
        this.botPoseYMeters = botPoseYMeters;
        this.botPoseHeadingRadians = botPoseHeadingRadians;
        this.rejectionReason = rejectionReason;
    }

    /** Builds the only actionable representation from a raw diagnostic sample. */
    public static LimelightObservation fromRaw(LimelightRawSample raw,
                                                double maxStalenessMs,
                                                int expectedTagId,
                                                int expectedPipelineIndex) {
        if (raw == null) {
            return rejected(Quality.INVALID, System.nanoTime(), "RAW_SAMPLE_NULL");
        }
        if (!raw.devicePresent) {
            return rejected(Quality.DEVICE_ABSENT, raw.pollTimestampNanos, "DEVICE_ABSENT");
        }
        if (!raw.deviceConnected) {
            return rejected(Quality.DEVICE_DISCONNECTED, raw.pollTimestampNanos,
                    "DEVICE_DISCONNECTED");
        }
        if (!raw.resultValid) {
            return rejected(Quality.INVALID, raw.pollTimestampNanos, "RESULT_INVALID");
        }
        if (!Double.isFinite(maxStalenessMs) || maxStalenessMs < 0.0) {
            return rejected(Quality.INVALID, raw.pollTimestampNanos, "MAX_STALENESS_INVALID");
        }
        if (!Double.isFinite(raw.stalenessMs) || raw.stalenessMs < 0.0
                || raw.stalenessMs > maxStalenessMs) {
            return rejected(Quality.STALE, raw.pollTimestampNanos,
                    "STALENESS_INVALID_OR_EXPIRED");
        }
        if (expectedPipelineIndex < 0 || raw.pipelineIndex != expectedPipelineIndex) {
            return rejected(Quality.WRONG_PIPELINE, raw.pollTimestampNanos,
                    "UNEXPECTED_PIPELINE");
        }
        if (raw.tagId < 0) {
            return rejected(Quality.NO_TARGET, raw.pollTimestampNanos, "NO_FIDUCIAL");
        }
        if (expectedTagId < 0 || raw.tagId != expectedTagId) {
            return rejected(Quality.WRONG_TAG, raw.pollTimestampNanos, "UNEXPECTED_TAG");
        }
        if (!allFinite(raw.txDegrees, raw.tyDegrees, raw.totalLatencyMs)
                || raw.totalLatencyMs < 0.0) {
            return rejected(Quality.INVALID, raw.pollTimestampNanos, "TARGET_NUMERIC_INVALID");
        }
        if (raw.controlHubTimestampMillis <= 0L || raw.fiducialCount <= 0
                || raw.tagFamily.isEmpty()) {
            return rejected(Quality.INVALID, raw.pollTimestampNanos,
                    "TARGET_METADATA_INVALID");
        }
        if (!Double.isFinite(raw.targetAreaPercent)
                || raw.targetAreaPercent <= 0.0 || raw.targetAreaPercent > 100.0
                || Math.abs(raw.txDegrees) > 180.0 || Math.abs(raw.tyDegrees) > 90.0) {
            return rejected(Quality.OUT_OF_BOUNDS, raw.pollTimestampNanos,
                    "TARGET_PHYSICALLY_IMPOSSIBLE");
        }
        if (raw.hasBotPose && !allFinite(raw.botPoseXMeters, raw.botPoseYMeters,
                raw.botPoseHeadingRadians)) {
            return rejected(Quality.INVALID, raw.pollTimestampNanos,
                    "BOTPOSE_NUMERIC_INVALID");
        }

        return new LimelightObservation(Quality.VALID, raw.pollTimestampNanos,
                raw.controlHubTimestampMillis, raw.pipelineIndex, raw.fiducialCount,
                raw.tagId, raw.tagFamily, raw.txDegrees, raw.tyDegrees,
                raw.targetAreaPercent, raw.stalenessMs, raw.totalLatencyMs,
                raw.hasBotPose,
                raw.hasBotPose ? raw.botPoseXMeters : 0.0,
                raw.hasBotPose ? raw.botPoseYMeters : 0.0,
                raw.hasBotPose ? Angles.normalizeRadians(raw.botPoseHeadingRadians) : 0.0,
                "");
    }

    public static LimelightObservation rejected(Quality quality, long timestampNanos,
                                                 String rejectionReason) {
        if (quality == null || quality == Quality.VALID) {
            throw new IllegalArgumentException("Rejected observation requires non-VALID quality");
        }
        return new LimelightObservation(quality, timestampNanos, 0L, -1, 0, -1, "",
                0.0, 0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0,
                rejectionReason == null ? "" : rejectionReason);
    }

    public boolean isUsable() {
        return quality == Quality.VALID;
    }

    private static boolean allFinite(double... values) {
        for (double value : values) {
            if (!Double.isFinite(value)) {
                return false;
            }
        }
        return true;
    }
}
