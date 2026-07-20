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
        INVALID
    }

    public final Quality quality;
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
    public final String rejectionReason;

    private LimelightObservation(Quality quality, long timestampNanos, int tagId,
                                 double txDegrees, double tyDegrees,
                                 double stalenessMs, double totalLatencyMs,
                                 boolean hasBotPose, double botPoseXMeters,
                                 double botPoseYMeters, double botPoseHeadingRadians,
                                 String rejectionReason) {
        this.quality = quality;
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
        this.rejectionReason = rejectionReason;
    }

    /** Builds the only actionable representation from a raw diagnostic sample. */
    public static LimelightObservation fromRaw(LimelightRawSample raw,
                                                double maxStalenessMs,
                                                int expectedTagId) {
        if (raw == null) {
            return rejected(Quality.INVALID, System.nanoTime(), "RAW_SAMPLE_NULL");
        }
        if (!raw.devicePresent) {
            return rejected(Quality.DEVICE_ABSENT, raw.timestampNanos, "DEVICE_ABSENT");
        }
        if (!raw.resultValid) {
            return rejected(Quality.INVALID, raw.timestampNanos, "RESULT_INVALID");
        }
        if (!Double.isFinite(maxStalenessMs) || maxStalenessMs < 0.0) {
            return rejected(Quality.INVALID, raw.timestampNanos, "MAX_STALENESS_INVALID");
        }
        if (!Double.isFinite(raw.stalenessMs) || raw.stalenessMs < 0.0
                || raw.stalenessMs > maxStalenessMs) {
            return rejected(Quality.STALE, raw.timestampNanos, "STALENESS_INVALID_OR_EXPIRED");
        }
        if (raw.tagId < 0) {
            return rejected(Quality.NO_TARGET, raw.timestampNanos, "NO_FIDUCIAL");
        }
        if (raw.tagId != expectedTagId) {
            return rejected(Quality.WRONG_TAG, raw.timestampNanos, "UNEXPECTED_TAG");
        }
        if (!allFinite(raw.txDegrees, raw.tyDegrees, raw.totalLatencyMs)
                || raw.totalLatencyMs < 0.0) {
            return rejected(Quality.INVALID, raw.timestampNanos, "TARGET_NUMERIC_INVALID");
        }
        if (raw.hasBotPose && !allFinite(raw.botPoseXMeters, raw.botPoseYMeters,
                raw.botPoseHeadingRadians)) {
            return rejected(Quality.INVALID, raw.timestampNanos, "BOTPOSE_NUMERIC_INVALID");
        }

        return new LimelightObservation(Quality.VALID, raw.timestampNanos, raw.tagId,
                raw.txDegrees, raw.tyDegrees, raw.stalenessMs, raw.totalLatencyMs,
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
        return new LimelightObservation(quality, timestampNanos, -1,
                0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0,
                rejectionReason == null ? "" : rejectionReason);
    }

    public boolean isUsable() {
        return quality == Quality.VALID;
    }

    /** Pure quality-only helper retained for tests and simple callers. */
    public static Quality classify(boolean devicePresent, boolean resultValid,
                                   double stalenessMs, double maxStalenessMs,
                                   int detectedTagId, int expectedTagId) {
        if (!devicePresent) {
            return Quality.DEVICE_ABSENT;
        }
        if (!resultValid || !Double.isFinite(maxStalenessMs) || maxStalenessMs < 0.0) {
            return Quality.INVALID;
        }
        if (!Double.isFinite(stalenessMs) || stalenessMs < 0.0
                || stalenessMs > maxStalenessMs) {
            return Quality.STALE;
        }
        if (detectedTagId < 0) {
            return Quality.NO_TARGET;
        }
        if (detectedTagId != expectedTagId) {
            return Quality.WRONG_TAG;
        }
        return Quality.VALID;
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
