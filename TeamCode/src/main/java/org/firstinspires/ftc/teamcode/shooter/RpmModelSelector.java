package org.firstinspires.ftc.teamcode.shooter;

import java.util.List;
import java.util.Locale;

/**
 * Selector de modelo por validación cruzada (DEC-012): evalúa lineal ->
 * cuadrático -> piecewise y devuelve el PRIMER (más simple) modelo que cumple
 * >=90% de aciertos en calibración y >=80% en el set retenido, con tolerancia
 * en RPM. Si ninguno cumple, devuelve piecewise marcado NO_MODEL_MET_CRITERIA
 * (fail-visible, nunca fail-silent).
 */
public final class RpmModelSelector {

    public static final double MIN_CALIBRATION_HIT_RATE = 0.9;
    public static final double MIN_HOLDOUT_HIT_RATE = 0.8;

    public static final class Result {
        public final RpmModel model;
        public final String rationale;
        public final double calibrationHitRate;
        public final double holdoutHitRate;
        public final boolean metCriteria;

        Result(RpmModel model, String rationale,
               double calibrationHitRate, double holdoutHitRate, boolean metCriteria) {
            this.model = model;
            this.rationale = rationale;
            this.calibrationHitRate = calibrationHitRate;
            this.holdoutHitRate = holdoutHitRate;
            this.metCriteria = metCriteria;
        }
    }

    private RpmModelSelector() {
    }

    public static Result select(List<ShotSample> calibration, List<ShotSample> holdout,
                                double toleranceRpm) {
        if (calibration == null || calibration.isEmpty() || holdout == null || holdout.isEmpty()) {
            throw new IllegalArgumentException(
                    "select requiere datasets de calibración y retenido no vacíos");
        }
        if (!(toleranceRpm > 0)) {
            throw new IllegalArgumentException("toleranceRpm debe ser > 0: " + toleranceRpm);
        }

        // Del más simple al más flexible (DEC-012: "el más simple que cumpla").
        RpmModel[] candidates = new RpmModel[3];
        String[] names = {"linear", "quadratic", "piecewise"};
        candidates[0] = tryFitLinear(calibration);
        candidates[1] = tryFitQuadratic(calibration);
        candidates[2] = PiecewiseLinearRpmModel.of(calibration);

        Result fallback = null;
        for (int i = 0; i < candidates.length; i++) {
            RpmModel model = candidates[i];
            if (model == null) {
                continue;
            }
            double calibrationRate = hitRate(model, calibration, toleranceRpm);
            double holdoutRate = hitRate(model, holdout, toleranceRpm);
            boolean met = calibrationRate >= MIN_CALIBRATION_HIT_RATE
                    && holdoutRate >= MIN_HOLDOUT_HIT_RATE;
            if (met) {
                return new Result(model, String.format(Locale.US,
                        "%s cumple DEC-012 (cal %.0f%%, holdout %.0f%%): %s",
                        names[i], calibrationRate * 100, holdoutRate * 100, model.describe()),
                        calibrationRate, holdoutRate, true);
            }
            if (i == candidates.length - 1) {
                fallback = new Result(model, String.format(Locale.US,
                        "NO_MODEL_MET_CRITERIA: ningún modelo cumple DEC-012; piecewise "
                                + "(cal %.0f%%, holdout %.0f%%) devuelto como mejor esfuerzo: %s",
                        calibrationRate * 100, holdoutRate * 100, model.describe()),
                        calibrationRate, holdoutRate, false);
            }
        }
        return fallback;
    }

    private static RpmModel tryFitLinear(List<ShotSample> calibration) {
        try {
            return LinearRpmModel.fit(calibration);
        } catch (IllegalArgumentException notEnoughData) {
            return null;
        }
    }

    private static RpmModel tryFitQuadratic(List<ShotSample> calibration) {
        try {
            return QuadraticRpmModel.fit(calibration);
        } catch (IllegalArgumentException notEnoughData) {
            return null;
        }
    }

    private static double hitRate(RpmModel model, List<ShotSample> samples, double toleranceRpm) {
        int hits = 0;
        for (ShotSample sample : samples) {
            if (Math.abs(model.rpmForDistance(sample.distanceInches) - sample.rpm) <= toleranceRpm) {
                hits++;
            }
        }
        return (double) hits / samples.size();
    }
}
