package org.firstinspires.ftc.teamcode.localization;

/**
 * Pose inmutable y neutral (MP-02): solo doubles, sin tipos de Road Runner ni
 * Pedro, para que MP-04/MP-05 no importen APIs de ningún localizador.
 *
 * Frame: el de campo de Road Runner — pulgadas, +X hacia adelante desde la pose
 * inicial del OpMode, heading CCW-positivo en radianes normalizado a (-PI, PI].
 * Cuando Pedro tome ownership (DEC-034) el adapter conserva este contrato.
 *
 * resetEpoch se incrementa en cada setPose/resetHeading: un consumidor que vea
 * cambiar el epoch debe descartar continuidad (la pose saltó a propósito).
 */
public final class PoseSnapshot {

    public final double xInches;
    public final double yInches;
    public final double headingRadians;
    public final double vxInchesPerSec;
    public final double vyInchesPerSec;
    public final double omegaRadiansPerSec;
    public final long timestampNanos;
    public final int resetEpoch;
    public final PoseQuality quality;

    public PoseSnapshot(double xInches, double yInches, double headingRadians,
                        double vxInchesPerSec, double vyInchesPerSec, double omegaRadiansPerSec,
                        long timestampNanos, int resetEpoch, PoseQuality quality) {
        if (quality == null) {
            throw new IllegalArgumentException("quality no puede ser null");
        }
        this.xInches = xInches;
        this.yInches = yInches;
        this.headingRadians = headingRadians;
        this.vxInchesPerSec = vxInchesPerSec;
        this.vyInchesPerSec = vyInchesPerSec;
        this.omegaRadiansPerSec = omegaRadiansPerSec;
        this.timestampNanos = timestampNanos;
        this.resetEpoch = resetEpoch;
        this.quality = quality;
    }

    /** Estado inicial fail-closed: todo cero y calidad UNINITIALIZED. */
    public static PoseSnapshot uninitialized(long timestampNanos) {
        return new PoseSnapshot(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                timestampNanos, 0, PoseQuality.UNINITIALIZED);
    }

    /** Solo una pose con odometría viva o fusión sana alimenta consumidores. */
    public boolean isUsable() {
        return quality == PoseQuality.ODOMETRY_ONLY || quality == PoseQuality.FUSED_GOOD;
    }
}
