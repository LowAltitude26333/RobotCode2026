package org.firstinspires.ftc.teamcode.localization;

import org.firstinspires.ftc.teamcode.util.Angles;

/**
 * Pose inmutable y neutral (MP-02): solo doubles, sin tipos de Road Runner ni
 * Pedro, para que MP-04/MP-05 no importen APIs de ningún localizador.
 *
 * Frame de POSICIÓN: el de campo de Road Runner — pulgadas, +X hacia adelante
 * desde la pose inicial del OpMode, heading CCW-positivo en radianes
 * normalizado a (-PI, PI].
 *
 * Frame de VELOCIDAD: ¡ROBOT, no campo! updatePoseEstimate() de RR devuelve la
 * velocidad robot-céntrica (el quickstart la nombra "robotVelRobot"): vx es
 * adelante del robot, vy es izquierda del robot. Un consumidor que necesite
 * velocidad de campo debe rotarla por headingRadians.
 *
 * Cuando Pedro tome ownership (DEC-034) el adapter conserva este contrato.
 *
 * resetEpoch se incrementa en cada setPose/resetHeading: un consumidor que vea
 * cambiar el epoch debe descartar continuidad (la pose saltó a propósito).
 */
public final class PoseSnapshot {

    public final double xInches;
    public final double yInches;
    public final double headingRadians;
    /** Velocidad en frame del ROBOT: +adelante (ver javadoc de la clase). */
    public final double vxInchesPerSec;
    /** Velocidad en frame del ROBOT: +izquierda (ver javadoc de la clase). */
    public final double vyInchesPerSec;
    public final double omegaRadiansPerSec;
    public final long timestampNanos;
    public final int resetEpoch;
    public final PoseQuality quality;
    /** Razón segura para telemetría; vacía cuando no hay rechazo. */
    public final String rejectionReason;

    private PoseSnapshot(double xInches, double yInches, double headingRadians,
                         double vxInchesPerSec, double vyInchesPerSec, double omegaRadiansPerSec,
                         long timestampNanos, int resetEpoch, PoseQuality quality,
                         String rejectionReason) {
        this.xInches = xInches;
        this.yInches = yInches;
        this.headingRadians = headingRadians;
        this.vxInchesPerSec = vxInchesPerSec;
        this.vyInchesPerSec = vyInchesPerSec;
        this.omegaRadiansPerSec = omegaRadiansPerSec;
        this.timestampNanos = timestampNanos;
        this.resetEpoch = resetEpoch;
        this.quality = quality;
        this.rejectionReason = rejectionReason;
    }

    /**
     * Factory fail-closed. Nunca publica números no finitos y normaliza heading
     * al contrato (-PI, PI], incluyendo -PI -> +PI.
     */
    public static PoseSnapshot of(double xInches, double yInches, double headingRadians,
                                  double vxInchesPerSec, double vyInchesPerSec,
                                  double omegaRadiansPerSec, long timestampNanos,
                                  int resetEpoch, PoseQuality quality) {
        if (quality == null) {
            return invalid(timestampNanos, resetEpoch, "QUALITY_NULL");
        }
        if (!allFinite(xInches, yInches, headingRadians,
                vxInchesPerSec, vyInchesPerSec, omegaRadiansPerSec)) {
            return invalid(timestampNanos, resetEpoch, "NON_FINITE_POSE_OR_VELOCITY");
        }
        return new PoseSnapshot(xInches, yInches, Angles.normalizeRadians(headingRadians),
                vxInchesPerSec, vyInchesPerSec, omegaRadiansPerSec,
                timestampNanos, resetEpoch, quality, "");
    }

    /** Estado inicial fail-closed: todo cero y calidad UNINITIALIZED. */
    public static PoseSnapshot uninitialized(long timestampNanos) {
        return new PoseSnapshot(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                timestampNanos, 0, PoseQuality.UNINITIALIZED, "POSE_NOT_UPDATED");
    }

    /** Estado inválido sanitizado: conserva epoch/razón, nunca NaN/Inf. */
    public static PoseSnapshot invalid(long timestampNanos, int resetEpoch, String reason) {
        String safeReason = reason == null || reason.trim().isEmpty() ? "INVALID" : reason;
        return new PoseSnapshot(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                timestampNanos, resetEpoch, PoseQuality.INVALID, safeReason);
    }

    /** Solo una pose con odometría viva o fusión sana alimenta consumidores. */
    public boolean isUsable() {
        return quality == PoseQuality.ODOMETRY_ONLY || quality == PoseQuality.FUSED_GOOD;
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
