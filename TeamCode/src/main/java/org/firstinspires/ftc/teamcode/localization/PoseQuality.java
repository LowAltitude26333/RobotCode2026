package org.firstinspires.ftc.teamcode.localization;

/**
 * Calidad de la pose publicada (MP-02). El orden refleja el ciclo de vida:
 * sin inicializar -> solo odometría -> fusionada -> degradada -> inválida.
 */
public enum PoseQuality {
    UNINITIALIZED,
    ODOMETRY_ONLY,
    FUSED_GOOD,
    DEGRADED,
    INVALID
}
