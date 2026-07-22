package org.firstinspires.ftc.teamcode.vision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class LimelightObservationTest {

    private static final long TIME = 123L;
    private static final int EXPECTED_TAG = 20;
    private static final int EXPECTED_PIPELINE = 0;

    @Test
    public void finiteExpectedTagIsUsableAndHeadingUsesProjectConvention() {
        LimelightObservation observation = LimelightObservation.fromRaw(
                raw(EXPECTED_TAG, 1.5, -2.5, 10, 12, true, 1, 2, -Math.PI),
                100, EXPECTED_TAG, EXPECTED_PIPELINE);

        assertTrue(observation.isUsable());
        assertEquals(LimelightObservation.Quality.VALID, observation.quality);
        assertEquals(Math.PI, observation.botPoseHeadingRadians, 0.0);
        assertEquals("", observation.rejectionReason);
    }

    @Test
    public void rejectedObservationAlwaysHasFiniteZeroActionChannel() {
        LimelightRawSample[] badSamples = {
                raw(EXPECTED_TAG, Double.NaN, 0, 10, 12, false, 0, 0, 0),
                raw(EXPECTED_TAG, 0, Double.POSITIVE_INFINITY, 10, 12, false, 0, 0, 0),
                raw(EXPECTED_TAG, 0, 0, Double.NaN, 12, false, 0, 0, 0),
                raw(EXPECTED_TAG, 0, 0, 10, Double.NEGATIVE_INFINITY, false, 0, 0, 0),
                raw(EXPECTED_TAG, 0, 0, 10, 12, true, Double.NaN, 0, 0),
                raw(EXPECTED_TAG, 0, 0, 10, 12, true, 0, 0, Double.POSITIVE_INFINITY)
        };

        for (LimelightRawSample raw : badSamples) {
            LimelightObservation observation = LimelightObservation.fromRaw(
                    raw, 100, EXPECTED_TAG, EXPECTED_PIPELINE);
            assertFalse(observation.isUsable());
            assertActionChannelZero(observation);
        }
    }

    @Test
    public void invalidFreshnessConfigurationFailsClosed() {
        LimelightObservation observation = LimelightObservation.fromRaw(
                raw(EXPECTED_TAG, 0, 0, 10, 12, false, 0, 0, 0),
                Double.POSITIVE_INFINITY, EXPECTED_TAG, EXPECTED_PIPELINE);
        assertEquals(LimelightObservation.Quality.INVALID, observation.quality);
        assertActionChannelZero(observation);
    }

    @Test
    public void absentStaleMissingAndWrongTagHaveDistinctQuality() {
        LimelightRawSample absent = LimelightRawSample.unavailable(false, false, TIME);
        assertEquals(LimelightObservation.Quality.DEVICE_ABSENT,
                LimelightObservation.fromRaw(absent, 100, EXPECTED_TAG,
                        EXPECTED_PIPELINE).quality);
        assertEquals(LimelightObservation.Quality.STALE,
                LimelightObservation.fromRaw(
                        raw(EXPECTED_TAG, 0, 0, 101, 0, false, 0, 0, 0),
                        100, EXPECTED_TAG, EXPECTED_PIPELINE).quality);
        assertEquals(LimelightObservation.Quality.NO_TARGET,
                LimelightObservation.fromRaw(
                        raw(-1, 0, 0, 1, 0, false, 0, 0, 0),
                        100, EXPECTED_TAG, EXPECTED_PIPELINE).quality);
        assertEquals(LimelightObservation.Quality.WRONG_TAG,
                LimelightObservation.fromRaw(
                        raw(7, 0, 0, 1, 0, false, 0, 0, 0),
                        100, EXPECTED_TAG, EXPECTED_PIPELINE).quality);
    }

    @Test
    public void disconnectedWrongPipelineAndImpossibleGeometryFailClosed() {
        LimelightRawSample disconnected = new LimelightRawSample(
                true, false, true, TIME, 456L, EXPECTED_PIPELINE,
                1, EXPECTED_TAG, "36h11", 0, 0, 1,
                1, 1, false, 0, 0, 0);
        assertEquals(LimelightObservation.Quality.DEVICE_DISCONNECTED,
                LimelightObservation.fromRaw(disconnected, 100, EXPECTED_TAG,
                        EXPECTED_PIPELINE).quality);

        LimelightRawSample wrongPipeline = raw(EXPECTED_TAG, 0, 0,
                1, 1, false, 0, 0, 0, EXPECTED_PIPELINE + 1, 1);
        assertEquals(LimelightObservation.Quality.WRONG_PIPELINE,
                LimelightObservation.fromRaw(wrongPipeline, 100, EXPECTED_TAG,
                        EXPECTED_PIPELINE).quality);

        LimelightRawSample impossible = raw(EXPECTED_TAG, 181, 0,
                1, 1, false, 0, 0, 0);
        LimelightObservation observation = LimelightObservation.fromRaw(
                impossible, 100, EXPECTED_TAG, EXPECTED_PIPELINE);
        assertEquals(LimelightObservation.Quality.OUT_OF_BOUNDS, observation.quality);
        assertActionChannelZero(observation);
    }

    private static LimelightRawSample raw(int tagId, double tx, double ty,
                                          double staleness, double latency,
                                          boolean hasBotPose, double x, double y,
                                          double heading) {
        return raw(tagId, tx, ty, staleness, latency, hasBotPose, x, y,
                heading, EXPECTED_PIPELINE, tagId < 0 ? 0.0 : 1.0);
    }

    private static LimelightRawSample raw(int tagId, double tx, double ty,
                                          double staleness, double latency,
                                          boolean hasBotPose, double x, double y,
                                          double heading, int pipeline, double area) {
        return new LimelightRawSample(true, true, true, TIME, 456L, pipeline,
                tagId < 0 ? 0 : 1, tagId, tagId < 0 ? "" : "36h11",
                tx, ty, area, staleness, latency, hasBotPose, x, y, heading);
    }

    private static void assertActionChannelZero(LimelightObservation observation) {
        assertEquals(-1, observation.tagId);
        assertEquals(-1, observation.pipelineIndex);
        assertEquals(0, observation.fiducialCount);
        assertEquals("", observation.tagFamily);
        assertEquals(0.0, observation.txDegrees, 0.0);
        assertEquals(0.0, observation.tyDegrees, 0.0);
        assertEquals(0.0, observation.targetAreaPercent, 0.0);
        assertEquals(0.0, observation.stalenessMs, 0.0);
        assertEquals(0.0, observation.totalLatencyMs, 0.0);
        assertFalse(observation.hasBotPose);
        assertEquals(0.0, observation.botPoseXMeters, 0.0);
        assertEquals(0.0, observation.botPoseYMeters, 0.0);
        assertEquals(0.0, observation.botPoseHeadingRadians, 0.0);
    }
}
