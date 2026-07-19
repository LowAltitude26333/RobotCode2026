package org.firstinspires.ftc.teamcode.shooter;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;

/**
 * Modelo RPM-por-distancia (MP-06/DEC-012). Toda implementación DEBE devolver
 * valores ya clampeados con {@link #clampRpm}: es imposible obtener una RPM
 * insegura de esta API, sin importar el dataset.
 */
public interface RpmModel {

    /** RPM objetivo para la distancia dada, siempre en [0, SHOOTER_MAX_SAFE_RPM]. */
    double rpmForDistance(double distanceInches);

    /** Descripción corta para telemetría/log, p.ej. "linear(m=20.0, b=2000.0)". */
    String describe();

    /** Clamp de seguridad compartido; el clamp vive en el modelo, no en el consumidor. */
    static double clampRpm(double rpm) {
        if (!Double.isFinite(rpm)) {
            // Un modelo que produce NaN/Inf no puede energizar el shooter.
            return 0.0;
        }
        return Math.max(0.0, Math.min(LowAltitudeConstants.SHOOTER_MAX_SAFE_RPM, rpm));
    }
}
