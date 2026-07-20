package org.firstinspires.ftc.teamcode.shooter;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Selecciona el candidato más simple que pase el gate físico DEC-012.
 * El ajuste matemático y la certificación son fases distintas: ningún porcentaje
 * calculado sobre los mismos puntos usados para ajustar certifica un shooter.
 */
public final class RpmModelSelector {

    public static final int MIN_TRIALS_PER_GROUP = 10;
    public static final int MIN_SESSIONS_PER_DISTANCE = 2;
    public static final double MIN_CALIBRATION_HIT_RATE = 0.9;
    public static final double MIN_HOLDOUT_HIT_RATE = 0.8;
    public static final double MAX_TARGET_RPM_ERROR = 1.0;
    public static final double MAX_MEASURED_RPM_ERROR = 100.0;
    public static final long MIN_RPM_READY_HOLD_MS = 250;

    public static final class Result {
        /** Siempre no-null; devuelve cero RPM cuando ningún candidato está certificado. */
        public final RpmModel model;
        /** null cuando ningún candidato pasó el gate. */
        public final RpmModelKind kind;
        public final String rationale;
        public final boolean metCriteria;

        private Result(RpmModel model, RpmModelKind kind,
                       String rationale, boolean metCriteria) {
            this.model = model;
            this.kind = kind;
            this.rationale = rationale;
            this.metCriteria = metCriteria;
        }
    }

    private static final class GroupKey {
        final ShotDatasetRole role;
        final String sessionId;
        final String distanceGroupId;

        GroupKey(ShotDatasetRole role, String sessionId, String distanceGroupId) {
            this.role = role;
            this.sessionId = sessionId;
            this.distanceGroupId = distanceGroupId;
        }

        @Override
        public boolean equals(Object other) {
            if (!(other instanceof GroupKey)) {
                return false;
            }
            GroupKey that = (GroupKey) other;
            return role == that.role
                    && sessionId.equals(that.sessionId)
                    && distanceGroupId.equals(that.distanceGroupId);
        }

        @Override
        public int hashCode() {
            int result = role.hashCode();
            result = 31 * result + sessionId.hashCode();
            return 31 * result + distanceGroupId.hashCode();
        }
    }

    private static final class DistanceKey {
        final ShotDatasetRole role;
        final String distanceGroupId;

        DistanceKey(ShotDatasetRole role, String distanceGroupId) {
            this.role = role;
            this.distanceGroupId = distanceGroupId;
        }

        @Override
        public boolean equals(Object other) {
            if (!(other instanceof DistanceKey)) {
                return false;
            }
            DistanceKey that = (DistanceKey) other;
            return role == that.role && distanceGroupId.equals(that.distanceGroupId);
        }

        @Override
        public int hashCode() {
            return 31 * role.hashCode() + distanceGroupId.hashCode();
        }
    }

    private static final class GateResult {
        final boolean passed;
        final String reason;

        GateResult(boolean passed, String reason) {
            this.passed = passed;
            this.reason = reason;
        }
    }

    private static final RpmModel ZERO_MODEL = new RpmModel() {
        @Override
        public double rpmForDistance(double distanceInches) {
            return 0.0;
        }

        @Override
        public String describe() {
            return "unavailable(zero-rpm)";
        }
    };

    private RpmModelSelector() {
    }

    public static Result select(List<ShotSample> calibration,
                                Map<RpmModelKind, List<ShotTrial>> trialsByModelKind) {
        if (calibration == null || calibration.isEmpty()) {
            throw new IllegalArgumentException("select requiere calibración no vacía");
        }
        for (ShotSample sample : calibration) {
            if (sample == null) {
                throw new IllegalArgumentException("calibración contiene una muestra null");
            }
        }

        Map<RpmModelKind, RpmModel> candidates = fitCandidates(calibration);
        List<String> failures = new ArrayList<>();
        for (RpmModelKind kind : RpmModelKind.values()) {
            RpmModel model = candidates.get(kind);
            if (model == null) {
                failures.add(kind + ": datos insuficientes/degenerados para ajustar");
                continue;
            }
            List<ShotTrial> trials = trialsByModelKind == null
                    ? null : trialsByModelKind.get(kind);
            GateResult gate = validatePhysicalGate(kind, model, trials);
            if (gate.passed) {
                return new Result(model, kind,
                        kind + " cumple DEC-012: " + model.describe(), true);
            }
            failures.add(kind + ": " + gate.reason);
        }

        return new Result(ZERO_MODEL, null,
                "NO_MODEL_MET_CRITERIA: " + joinFailures(failures), false);
    }

    private static Map<RpmModelKind, RpmModel> fitCandidates(List<ShotSample> calibration) {
        Map<RpmModelKind, RpmModel> candidates = new EnumMap<>(RpmModelKind.class);
        try {
            candidates.put(RpmModelKind.LINEAR, LinearRpmModel.fit(calibration));
        } catch (IllegalArgumentException ignored) {
            // Reportado como candidato no ajustable durante select().
        }
        try {
            candidates.put(RpmModelKind.QUADRATIC, QuadraticRpmModel.fit(calibration));
        } catch (IllegalArgumentException ignored) {
            // Reportado como candidato no ajustable durante select().
        }
        try {
            candidates.put(RpmModelKind.PIECEWISE, PiecewiseLinearRpmModel.of(calibration));
        } catch (IllegalArgumentException ignored) {
            // Reportado como candidato no ajustable durante select().
        }
        return candidates;
    }

    private static GateResult validatePhysicalGate(RpmModelKind expectedKind,
                                                   RpmModel model,
                                                   List<ShotTrial> trials) {
        if (trials == null || trials.isEmpty()) {
            return new GateResult(false, "sin intentos físicos");
        }

        Map<GroupKey, List<ShotTrial>> groups = new HashMap<>();
        Map<DistanceKey, Set<String>> sessionsByDistance = new HashMap<>();
        Set<String> allSessions = new HashSet<>();
        boolean hasCalibration = false;
        boolean hasHoldout = false;
        for (ShotTrial trial : trials) {
            if (trial == null) {
                return new GateResult(false, "lista contiene un intento null");
            }
            if (trial.modelKind != expectedKind) {
                return new GateResult(false, "intento etiquetado " + trial.modelKind
                        + " apareció en dataset " + expectedKind);
            }
            double expectedRpm = model.rpmForDistance(trial.distanceInches);
            if (expectedRpm <= 0.0
                    || Math.abs(trial.targetRpm - expectedRpm) > MAX_TARGET_RPM_ERROR) {
                return new GateResult(false, trial.sessionId + "/" + trial.distanceGroupId
                        + " no usó el RPM del candidato");
            }
            if (Math.abs(trial.measuredRpmAtFeed - trial.targetRpm)
                    > MAX_MEASURED_RPM_ERROR) {
                return new GateResult(false, trial.sessionId + "/" + trial.distanceGroupId
                        + " alimentó fuera de ±100 RPM");
            }
            if (trial.rpmReadyHoldMs < MIN_RPM_READY_HOLD_MS) {
                return new GateResult(false, trial.sessionId + "/" + trial.distanceGroupId
                        + " sostuvo RPM " + trial.rpmReadyHoldMs + " ms; requiere >=250 ms");
            }
            hasCalibration |= trial.role == ShotDatasetRole.CALIBRATION;
            hasHoldout |= trial.role == ShotDatasetRole.HOLDOUT;
            allSessions.add(trial.sessionId);

            GroupKey groupKey = new GroupKey(
                    trial.role, trial.sessionId, trial.distanceGroupId);
            List<ShotTrial> group = groups.get(groupKey);
            if (group == null) {
                group = new ArrayList<>();
                groups.put(groupKey, group);
            }
            group.add(trial);

            DistanceKey distanceKey = new DistanceKey(trial.role, trial.distanceGroupId);
            Set<String> sessions = sessionsByDistance.get(distanceKey);
            if (sessions == null) {
                sessions = new HashSet<>();
                sessionsByDistance.put(distanceKey, sessions);
            }
            sessions.add(trial.sessionId);
        }

        if (!hasCalibration || !hasHoldout) {
            return new GateResult(false, "se requieren distancias CALIBRATION y HOLDOUT");
        }

        if (allSessions.size() < MIN_SESSIONS_PER_DISTANCE) {
            return new GateResult(false, "dataset requires at least two sessions");
        }

        for (Map.Entry<DistanceKey, Set<String>> entry : sessionsByDistance.entrySet()) {
            if (!entry.getValue().containsAll(allSessions)) {
                DistanceKey key = entry.getKey();
                return new GateResult(false, key.role + "/" + key.distanceGroupId
                        + " is missing from one or more sessions");
            }
        }

        for (Map.Entry<GroupKey, List<ShotTrial>> entry : groups.entrySet()) {
            GroupKey key = entry.getKey();
            List<ShotTrial> group = entry.getValue();
            if (group.size() < MIN_TRIALS_PER_GROUP) {
                return new GateResult(false, describe(key) + " tiene " + group.size()
                        + " intentos; requiere >=10");
            }
            int scored = 0;
            for (ShotTrial trial : group) {
                if (trial.scored()) {
                    scored++;
                }
            }
            double rate = (double) scored / group.size();
            double required = key.role == ShotDatasetRole.CALIBRATION
                    ? MIN_CALIBRATION_HIT_RATE : MIN_HOLDOUT_HIT_RATE;
            if (rate < required) {
                return new GateResult(false, describe(key) + " logró " + scored + "/"
                        + group.size() + "; requiere >=" + (int) Math.ceil(required * group.size()));
            }
        }
        return new GateResult(true, "todos los grupos cumplen");
    }

    private static String describe(GroupKey key) {
        return key.role + "/" + key.sessionId + "/" + key.distanceGroupId;
    }

    private static String joinFailures(List<String> failures) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < failures.size(); i++) {
            if (i > 0) {
                builder.append("; ");
            }
            builder.append(failures.get(i));
        }
        return builder.toString();
    }
}
