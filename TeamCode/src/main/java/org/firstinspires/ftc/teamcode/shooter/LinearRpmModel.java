package org.firstinspires.ftc.teamcode.shooter;

import java.util.List;
import java.util.Locale;

/** Ajuste rpm = m*d + b por mínimos cuadrados cerrados (DEC-012, candidato 1). */
public final class LinearRpmModel implements RpmModel {

    private final double slope;
    private final double intercept;

    private LinearRpmModel(double slope, double intercept) {
        this.slope = slope;
        this.intercept = intercept;
    }

    public static LinearRpmModel fit(List<ShotSample> samples) {
        if (samples == null || RpmModels.distinctDistanceCount(samples) < 2) {
            throw new IllegalArgumentException(
                    "LinearRpmModel.fit requiere >=2 distancias distintas");
        }
        int n = samples.size();
        double sumD = 0, sumR = 0, sumDD = 0, sumDR = 0;
        for (ShotSample s : samples) {
            sumD += s.distanceInches;
            sumR += s.rpm;
            sumDD += s.distanceInches * s.distanceInches;
            sumDR += s.distanceInches * s.rpm;
        }
        double denominator = n * sumDD - sumD * sumD;
        if (!Double.isFinite(denominator) || Math.abs(denominator) < 1e-12) {
            throw new IllegalArgumentException("Dataset lineal degenerado o fuera de rango numérico");
        }
        double slope = (n * sumDR - sumD * sumR) / denominator;
        double intercept = (sumR - slope * sumD) / n;
        if (!Double.isFinite(slope) || !Double.isFinite(intercept)) {
            throw new IllegalArgumentException("Ajuste lineal produjo coeficientes no finitos");
        }
        return new LinearRpmModel(slope, intercept);
    }

    @Override
    public double rpmForDistance(double distanceInches) {
        if (!RpmModel.isValidDistance(distanceInches)) {
            return 0.0;
        }
        return RpmModel.clampRpm(slope * distanceInches + intercept);
    }

    @Override
    public String describe() {
        return String.format(Locale.US, "linear(m=%.4f, b=%.1f)", slope, intercept);
    }
}
