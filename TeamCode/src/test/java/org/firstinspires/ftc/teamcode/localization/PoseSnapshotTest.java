package org.firstinspires.ftc.teamcode.localization;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class PoseSnapshotTest {

    @Test
    public void uninitializedIsAllZeroAndNotUsable() {
        PoseSnapshot snapshot = PoseSnapshot.uninitialized(123L);
        assertEquals(PoseQuality.UNINITIALIZED, snapshot.quality);
        assertEquals("POSE_NOT_UPDATED", snapshot.rejectionReason);
        assertEquals(123L, snapshot.timestampNanos);
        assertAllNumericZero(snapshot);
        assertFalse(snapshot.isUsable());
    }

    @Test
    public void usableFinitePoseCarriesRobotFrameVelocityAndNormalizesHeading() {
        PoseSnapshot snapshot = PoseSnapshot.of(12.5, -3.25, -Math.PI,
                4.0, -2.0, 0.75, 999L, 7, PoseQuality.ODOMETRY_ONLY);
        assertTrue(snapshot.isUsable());
        assertEquals(12.5, snapshot.xInches, 0.0);
        assertEquals(-3.25, snapshot.yInches, 0.0);
        assertEquals(Math.PI, snapshot.headingRadians, 0.0);
        assertEquals(4.0, snapshot.vxInchesPerSec, 0.0);
        assertEquals(-2.0, snapshot.vyInchesPerSec, 0.0);
        assertEquals(0.75, snapshot.omegaRadiansPerSec, 0.0);
        assertEquals(999L, snapshot.timestampNanos);
        assertEquals(7, snapshot.resetEpoch);
        assertEquals("", snapshot.rejectionReason);
    }

    @Test
    public void everyNonFinitePoseOrVelocityValueFailsClosed() {
        double[] bad = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};
        for (double value : bad) {
            assertRejected(PoseSnapshot.of(value, 0, 0, 0, 0, 0,
                    1, 2, PoseQuality.ODOMETRY_ONLY));
            assertRejected(PoseSnapshot.of(0, value, 0, 0, 0, 0,
                    1, 2, PoseQuality.ODOMETRY_ONLY));
            assertRejected(PoseSnapshot.of(0, 0, value, 0, 0, 0,
                    1, 2, PoseQuality.ODOMETRY_ONLY));
            assertRejected(PoseSnapshot.of(0, 0, 0, value, 0, 0,
                    1, 2, PoseQuality.ODOMETRY_ONLY));
            assertRejected(PoseSnapshot.of(0, 0, 0, 0, value, 0,
                    1, 2, PoseQuality.ODOMETRY_ONLY));
            assertRejected(PoseSnapshot.of(0, 0, 0, 0, 0, value,
                    1, 2, PoseQuality.ODOMETRY_ONLY));
        }
    }

    @Test
    public void nullQualityFailsClosed() {
        PoseSnapshot snapshot = PoseSnapshot.of(1, 2, 3, 4, 5, 6, 7, 8, null);
        assertRejected(snapshot);
        assertEquals("QUALITY_NULL", snapshot.rejectionReason);
    }

    private static void assertRejected(PoseSnapshot snapshot) {
        assertEquals(PoseQuality.INVALID, snapshot.quality);
        assertFalse(snapshot.isUsable());
        assertAllNumericZero(snapshot);
    }

    private static void assertAllNumericZero(PoseSnapshot snapshot) {
        assertEquals(0.0, snapshot.xInches, 0.0);
        assertEquals(0.0, snapshot.yInches, 0.0);
        assertEquals(0.0, snapshot.headingRadians, 0.0);
        assertEquals(0.0, snapshot.vxInchesPerSec, 0.0);
        assertEquals(0.0, snapshot.vyInchesPerSec, 0.0);
        assertEquals(0.0, snapshot.omegaRadiansPerSec, 0.0);
    }
}
