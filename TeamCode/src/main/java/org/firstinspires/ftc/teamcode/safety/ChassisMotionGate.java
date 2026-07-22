package org.firstinspires.ftc.teamcode.safety;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.localization.PoseSnapshot;

/**
 * Pure predicate for DEC-031 (feeder solo con robot estacionario) / FND-019. Reads the
 * robot-frame velocity that {@link PoseSnapshot} already carries and compares it against
 * the TBD-BLOCKING envelope in {@link LowAltitudeConstants}. Never owns or polls hardware.
 */
public final class ChassisMotionGate {

    private ChassisMotionGate() {
    }

    /** An unusable snapshot (no live odometry) fails closed: never within the shot envelope. */
    public static boolean isWithinShotEnvelope(PoseSnapshot snapshot) {
        if (snapshot == null || !snapshot.isUsable()) {
            return false;
        }
        double linearSpeedInchesPerSec =
                Math.hypot(snapshot.vxInchesPerSec, snapshot.vyInchesPerSec);
        return linearSpeedInchesPerSec <= LowAltitudeConstants.MAX_SHOT_LINEAR_SPEED_INCHES_PER_SEC
                && Math.abs(snapshot.omegaRadiansPerSec)
                        <= LowAltitudeConstants.MAX_SHOT_ANGULAR_SPEED_RADIANS_PER_SEC;
    }
}
