package org.firstinspires.ftc.teamcode.shooter;

/**
 * Intento físico inmutable para certificar un candidato RPM.
 * distanceGroupId es la etiqueta estable del punto planeado (p.ej. "D44");
 * distanceInches conserva la medición real sin usar igualdad de double para agrupar.
 */
public final class ShotTrial {
    public final String sessionId;
    public final String distanceGroupId;
    public final RpmModelKind modelKind;
    public final ShotDatasetRole role;
    public final double distanceInches;
    public final double targetRpm;
    public final double measuredRpmAtFeed;
    /** Tiempo continuo dentro de ±100 RPM antes de alimentar la pieza. */
    public final long rpmReadyHoldMs;
    public final ShotOutcome outcome;

    public ShotTrial(String sessionId, String distanceGroupId, RpmModelKind modelKind,
                     ShotDatasetRole role,
                     double distanceInches, double targetRpm, double measuredRpmAtFeed,
                     long rpmReadyHoldMs, ShotOutcome outcome) {
        if (sessionId == null || sessionId.trim().isEmpty()) {
            throw new IllegalArgumentException("sessionId no puede estar vacío");
        }
        if (distanceGroupId == null || distanceGroupId.trim().isEmpty()) {
            throw new IllegalArgumentException("distanceGroupId no puede estar vacío");
        }
        if (modelKind == null || role == null || outcome == null) {
            throw new IllegalArgumentException("modelKind, role y outcome son obligatorios");
        }
        if (!RpmModel.isValidDistance(distanceInches)
                || !Double.isFinite(targetRpm) || targetRpm < 0.0
                || !Double.isFinite(measuredRpmAtFeed) || measuredRpmAtFeed < 0.0
                || rpmReadyHoldMs < 0) {
            throw new IllegalArgumentException("ShotTrial requiere distancia/RPM finitas y no negativas");
        }
        this.sessionId = sessionId.trim();
        this.distanceGroupId = distanceGroupId.trim();
        this.modelKind = modelKind;
        this.role = role;
        this.distanceInches = distanceInches;
        this.targetRpm = targetRpm;
        this.measuredRpmAtFeed = measuredRpmAtFeed;
        this.rpmReadyHoldMs = rpmReadyHoldMs;
        this.outcome = outcome;
    }

    public boolean scored() {
        return outcome == ShotOutcome.SCORED;
    }
}
