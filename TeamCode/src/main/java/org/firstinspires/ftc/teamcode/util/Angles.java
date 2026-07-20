package org.firstinspires.ftc.teamcode.util;

/**
 * Matemática pura de ángulos (MP-02/MP-05). Sin dependencias Android/SDK para
 * que sea testeable en src/test. Semántica de normalización idéntica a la que
 * DriveSubsystem usaba internamente: rango (-PI, PI], con -PI normalizado a +PI.
 */
public final class Angles {

    private Angles() {
    }

    /** Normaliza a (-PI, PI]. */
    public static double normalizeRadians(double angle) {
        if (!Double.isFinite(angle)) {
            return 0.0;
        }
        double normalized = angle % (2.0 * Math.PI);
        if (normalized > Math.PI) {
            normalized -= 2.0 * Math.PI;
        } else if (normalized <= -Math.PI) {
            normalized += 2.0 * Math.PI;
        }
        return normalized;
    }

    /** Normaliza a (-180, 180]. */
    public static double normalizeDegrees(double angle) {
        return Math.toDegrees(normalizeRadians(Math.toRadians(angle)));
    }

    /**
     * Delta con signo más corto para ir de {@code fromRadians} a {@code toRadians},
     * en (-PI, PI]. Pieza base del wrapping de auto-aim de torreta (MP-05).
     */
    public static double shortestDelta(double fromRadians, double toRadians) {
        return normalizeRadians(toRadians - fromRadians);
    }
}
