package org.firstinspires.ftc.teamcode.vision;

/**
 * Observación inmutable de la Limelight (MP-03).
 *
 * Clase pura sin imports del SDK para que {@link #classify} sea testeable en
 * {@code src/test}. El subsystem es el único que construye instancias VALID;
 * cualquier condición dudosa produce una observación rechazada (fail-closed).
 *
 * Unidades: tx/ty en grados (convención Limelight, tx positivo a la derecha),
 * staleness/latencia en milisegundos, botpose en metros/radianes (frame de
 * cancha reportado por la Limelight; MP-04 define la reconciliación de frames).
 */
public final class LimelightObservation {

    public enum Quality {
        VALID,
        NO_TARGET,
        WRONG_TAG,
        STALE,
        DEVICE_ABSENT,
        INVALID
    }

    public final Quality quality;
    /** System.nanoTime() local al momento de capturar/clasificar la observación. */
    public final long timestampNanos;
    /** ID del fiducial usado; -1 cuando no hay tag utilizable. */
    public final int tagId;
    public final double txDegrees;
    public final double tyDegrees;
    public final double stalenessMs;
    /** captureLatency + targetingLatency + parseLatency reportadas por la Limelight. */
    public final double totalLatencyMs;
    public final boolean hasBotPose;
    public final double botPoseXMeters;
    public final double botPoseYMeters;
    public final double botPoseHeadingRadians;

    private LimelightObservation(Quality quality, long timestampNanos, int tagId,
                                 double txDegrees, double tyDegrees,
                                 double stalenessMs, double totalLatencyMs,
                                 boolean hasBotPose, double botPoseXMeters,
                                 double botPoseYMeters, double botPoseHeadingRadians) {
        this.quality = quality;
        this.timestampNanos = timestampNanos;
        this.tagId = tagId;
        this.txDegrees = txDegrees;
        this.tyDegrees = tyDegrees;
        this.stalenessMs = stalenessMs;
        this.totalLatencyMs = totalLatencyMs;
        this.hasBotPose = hasBotPose;
        this.botPoseXMeters = botPoseXMeters;
        this.botPoseYMeters = botPoseYMeters;
        this.botPoseHeadingRadians = botPoseHeadingRadians;
    }

    public static LimelightObservation valid(long timestampNanos, int tagId,
                                             double txDegrees, double tyDegrees,
                                             double stalenessMs, double totalLatencyMs,
                                             boolean hasBotPose, double botPoseXMeters,
                                             double botPoseYMeters, double botPoseHeadingRadians) {
        return new LimelightObservation(Quality.VALID, timestampNanos, tagId,
                txDegrees, tyDegrees, stalenessMs, totalLatencyMs,
                hasBotPose, botPoseXMeters, botPoseYMeters, botPoseHeadingRadians);
    }

    public static LimelightObservation rejected(Quality reason, long timestampNanos) {
        if (reason == Quality.VALID) {
            throw new IllegalArgumentException("rejected() requiere una Quality distinta de VALID");
        }
        return new LimelightObservation(reason, timestampNanos, -1,
                0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0);
    }

    /** Solo una observación VALID puede alimentar consumidores (auto-aim, fusión). */
    public boolean isUsable() {
        return quality == Quality.VALID;
    }

    /**
     * Reglas fail-closed de aceptación (orden estricto): dispositivo ausente,
     * resultado nulo/inválido, viejo, sin fiducial, tag equivocado y solo al
     * final VALID.
     *
     * @param detectedTagId -1 cuando no se detectó ningún fiducial; en otro caso
     *                      el ID del fiducial esperado si está presente, o el
     *                      primer ID detectado si ninguno coincide.
     */
    public static Quality classify(boolean devicePresent, boolean resultValid,
                                   double stalenessMs, double maxStalenessMs,
                                   int detectedTagId, int expectedTagId) {
        if (!devicePresent) {
            return Quality.DEVICE_ABSENT;
        }
        if (!resultValid) {
            return Quality.INVALID;
        }
        if (!(stalenessMs >= 0.0) || stalenessMs > maxStalenessMs) {
            // NaN o negativo también se rechaza: un reloj imposible no es evidencia fresca.
            return Quality.STALE;
        }
        if (detectedTagId < 0) {
            return Quality.NO_TARGET;
        }
        if (detectedTagId != expectedTagId) {
            return Quality.WRONG_TAG;
        }
        return Quality.VALID;
    }
}
