package org.firstinspires.ftc.teamcode.localization;

/**
 * Fuente única y neutral de pose (MP-02). Los consumidores (MP-04 fusión,
 * MP-05 auto-aim) dependen SOLO de esta interfaz, nunca de Road Runner ni de
 * Pedro directamente. Hoy la implementación activa es RoadRunnerPoseProvider;
 * cuando DEC-034 se ejecute, se sustituye por un adapter de Pedro sin tocar
 * a los consumidores.
 */
public interface PoseProvider {

    /** Nunca null; UNINITIALIZED hasta el primer ciclo de odometría. */
    PoseSnapshot getSnapshot();

    /**
     * Reposiciona la pose (frame/unidades de PoseSnapshot) e incrementa el
     * resetEpoch para que los consumidores descarten continuidad.
     */
    void setPose(double xInches, double yInches, double headingRadians);

    int getResetEpoch();
}
