package org.firstinspires.ftc.teamcode.localization;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Test;

public class PoseSnapshotTest {

    @Test
    public void uninitializedIsAllZeroAndNotUsable() {
        PoseSnapshot snapshot = PoseSnapshot.uninitialized(123L);
        assertEquals(PoseQuality.UNINITIALIZED, snapshot.quality);
        assertEquals(123L, snapshot.timestampNanos);
        assertEquals(0, snapshot.resetEpoch);
        assertEquals(0.0, snapshot.xInches, 0.0);
        assertEquals(0.0, snapshot.yInches, 0.0);
        assertEquals(0.0, snapshot.headingRadians, 0.0);
        assertEquals(0.0, snapshot.vxInchesPerSec, 0.0);
        assertEquals(0.0, snapshot.vyInchesPerSec, 0.0);
        assertEquals(0.0, snapshot.omegaRadiansPerSec, 0.0);
        assertFalse(snapshot.isUsable());
    }

    @Test
    public void usabilityPerQuality() {
        assertFalse(snapshotWith(PoseQuality.UNINITIALIZED).isUsable());
        assertTrue(snapshotWith(PoseQuality.ODOMETRY_ONLY).isUsable());
        assertTrue(snapshotWith(PoseQuality.FUSED_GOOD).isUsable());
        assertFalse(snapshotWith(PoseQuality.DEGRADED).isUsable());
        assertFalse(snapshotWith(PoseQuality.INVALID).isUsable());
    }

    @Test
    public void nullQualityRejected() {
        try {
            snapshotWith(null);
            fail("Se esperaba IllegalArgumentException con quality null");
        } catch (IllegalArgumentException expected) {
            // fail-closed: una pose sin calidad declarada no puede existir.
        }
    }

    @Test
    public void fieldsAreCarriedVerbatim() {
        PoseSnapshot snapshot = new PoseSnapshot(12.5, -3.25, 1.5,
                4.0, -2.0, 0.75, 999L, 7, PoseQuality.ODOMETRY_ONLY);
        assertEquals(12.5, snapshot.xInches, 0.0);
        assertEquals(-3.25, snapshot.yInches, 0.0);
        assertEquals(1.5, snapshot.headingRadians, 0.0);
        assertEquals(4.0, snapshot.vxInchesPerSec, 0.0);
        assertEquals(-2.0, snapshot.vyInchesPerSec, 0.0);
        assertEquals(0.75, snapshot.omegaRadiansPerSec, 0.0);
        assertEquals(999L, snapshot.timestampNanos);
        assertEquals(7, snapshot.resetEpoch);
    }

    private static PoseSnapshot snapshotWith(PoseQuality quality) {
        return new PoseSnapshot(1.0, 2.0, 0.5, 0.0, 0.0, 0.0, 42L, 3, quality);
    }
}
