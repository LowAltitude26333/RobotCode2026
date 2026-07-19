package org.firstinspires.ftc.teamcode.shooter;

import java.util.List;
import java.util.Locale;

/**
 * Ajuste rpm = a*d^2 + b*d + c (DEC-012, candidato 2; grado máximo 2).
 * Ecuaciones normales 3x3 resueltas por eliminación gaussiana con pivoteo
 * parcial — sin librerías nuevas (restricción de dependencias).
 */
public final class QuadraticRpmModel implements RpmModel {

    private final double a;
    private final double b;
    private final double c;

    private QuadraticRpmModel(double a, double b, double c) {
        this.a = a;
        this.b = b;
        this.c = c;
    }

    public static QuadraticRpmModel fit(List<ShotSample> samples) {
        if (samples == null || RpmModels.distinctDistanceCount(samples) < 3) {
            throw new IllegalArgumentException(
                    "QuadraticRpmModel.fit requiere >=3 distancias distintas");
        }
        // Ecuaciones normales para [a, b, c] con base [d^2, d, 1].
        double s0 = samples.size();
        double s1 = 0, s2 = 0, s3 = 0, s4 = 0;
        double t0 = 0, t1 = 0, t2 = 0;
        for (ShotSample s : samples) {
            double d = s.distanceInches;
            double d2 = d * d;
            s1 += d;
            s2 += d2;
            s3 += d2 * d;
            s4 += d2 * d2;
            t0 += s.rpm;
            t1 += d * s.rpm;
            t2 += d2 * s.rpm;
        }
        double[][] m = {
                {s4, s3, s2, t2},
                {s3, s2, s1, t1},
                {s2, s1, s0, t0},
        };
        double[] solution = solve3x3(m);
        return new QuadraticRpmModel(solution[0], solution[1], solution[2]);
    }

    private static double[] solve3x3(double[][] m) {
        for (int col = 0; col < 3; col++) {
            int pivot = col;
            for (int row = col + 1; row < 3; row++) {
                if (Math.abs(m[row][col]) > Math.abs(m[pivot][col])) {
                    pivot = row;
                }
            }
            if (Math.abs(m[pivot][col]) < 1e-12) {
                throw new IllegalArgumentException(
                        "Dataset degenerado: sistema de ecuaciones normales singular");
            }
            double[] tmp = m[col];
            m[col] = m[pivot];
            m[pivot] = tmp;
            for (int row = col + 1; row < 3; row++) {
                double factor = m[row][col] / m[col][col];
                for (int k = col; k < 4; k++) {
                    m[row][k] -= factor * m[col][k];
                }
            }
        }
        double[] x = new double[3];
        for (int row = 2; row >= 0; row--) {
            double sum = m[row][3];
            for (int k = row + 1; k < 3; k++) {
                sum -= m[row][k] * x[k];
            }
            x[row] = sum / m[row][row];
        }
        return x;
    }

    @Override
    public double rpmForDistance(double distanceInches) {
        return RpmModel.clampRpm(a * distanceInches * distanceInches + b * distanceInches + c);
    }

    @Override
    public String describe() {
        return String.format(Locale.US, "quadratic(a=%.6f, b=%.4f, c=%.1f)", a, b, c);
    }
}
