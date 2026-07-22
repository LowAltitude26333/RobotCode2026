package org.firstinspires.ftc.teamcode.safety;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.localization.PoseQuality;
import org.firstinspires.ftc.teamcode.localization.PoseSnapshot;
import org.junit.Test;

public class ChassisMotionGateTest {

    @Test
    public void nullSnapshotFailsClosed() {
        assertFalse(ChassisMotionGate.isWithinShotEnvelope(null));
    }

    @Test
    public void unusableSnapshotFailsClosed() {
        assertFalse(ChassisMotionGate.isWithinShotEnvelope(PoseSnapshot.uninitialized(1L)));
    }

    @Test
    public void stationaryUsableSnapshotIsWithinEnvelope() {
        PoseSnapshot snapshot = PoseSnapshot.of(0, 0, 0, 0, 0, 0, 1L, 0,
                PoseQuality.ODOMETRY_ONLY);
        assertTrue(ChassisMotionGate.isWithinShotEnvelope(snapshot));
    }

    @Test
    public void linearSpeedAtLimitIsWithinEnvelopeJustOverIsRejected() {
        double limit = LowAltitudeConstants.MAX_SHOT_LINEAR_SPEED_INCHES_PER_SEC;
        PoseSnapshot atLimit = PoseSnapshot.of(0, 0, 0, limit, 0, 0, 1L, 0,
                PoseQuality.ODOMETRY_ONLY);
        assertTrue(ChassisMotionGate.isWithinShotEnvelope(atLimit));

        PoseSnapshot overLimit = PoseSnapshot.of(0, 0, 0, limit + 0.01, 0, 0, 1L, 0,
                PoseQuality.ODOMETRY_ONLY);
        assertFalse(ChassisMotionGate.isWithinShotEnvelope(overLimit));
    }

    @Test
    public void combinedVxVyMagnitudeIsCheckedNotEachAxisIndependently() {
        double limit = LowAltitudeConstants.MAX_SHOT_LINEAR_SPEED_INCHES_PER_SEC;
        double component = limit * 0.8;
        PoseSnapshot snapshot = PoseSnapshot.of(0, 0, 0, component, component, 0, 1L, 0,
                PoseQuality.ODOMETRY_ONLY);
        assertFalse(ChassisMotionGate.isWithinShotEnvelope(snapshot));
    }

    @Test
    public void angularSpeedOverLimitIsRejectedRegardlessOfSign() {
        double limit = LowAltitudeConstants.MAX_SHOT_ANGULAR_SPEED_RADIANS_PER_SEC;
        PoseSnapshot positive = PoseSnapshot.of(0, 0, 0, 0, 0, limit + 0.01, 1L, 0,
                PoseQuality.ODOMETRY_ONLY);
        PoseSnapshot negative = PoseSnapshot.of(0, 0, 0, 0, 0, -(limit + 0.01), 1L, 0,
                PoseQuality.ODOMETRY_ONLY);
        assertFalse(ChassisMotionGate.isWithinShotEnvelope(positive));
        assertFalse(ChassisMotionGate.isWithinShotEnvelope(negative));
    }
}
