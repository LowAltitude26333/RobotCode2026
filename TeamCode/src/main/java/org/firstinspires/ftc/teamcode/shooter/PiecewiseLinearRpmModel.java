package org.firstinspires.ftc.teamcode.shooter;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

/**
 * Tabla piecewise-linear distancia -> RPM (DEC-012, candidato 3).
 *
 * Adaptado de HyperionBots WorldsRebuildTeleOp (FTC 18011), PrecomputeShooterLookupTable:
 * misma lógica de clamp-bajo-el-primer-punto / clamp-sobre-el-último /
 * interpolación t entre puntos vecinos. Diferencias locales: sin hood (ángulo
 * fijo), unidades en RPM, distancias duplicadas promediadas y sin tabla
 * precomputada (con <=10 puntos la búsqueda lineal basta).
 *
 * Fuera de rango SATURA al punto extremo: extrapolar un tiro es peor que
 * quedarse corto de forma predecible.
 */
public final class PiecewiseLinearRpmModel implements RpmModel {

    private final double[] distances;
    private final double[] rpms;

    private PiecewiseLinearRpmModel(double[] distances, double[] rpms) {
        this.distances = distances;
        this.rpms = rpms;
    }

    public static PiecewiseLinearRpmModel of(List<ShotSample> samples) {
        if (samples == null || samples.isEmpty()) {
            throw new IllegalArgumentException(
                    "PiecewiseLinearRpmModel.of requiere al menos 1 muestra");
        }
        List<ShotSample> sorted = new ArrayList<>(samples);
        Collections.sort(sorted, new Comparator<ShotSample>() {
            @Override
            public int compare(ShotSample first, ShotSample second) {
                return Double.compare(first.distanceInches, second.distanceInches);
            }
        });

        // Promediar RPM en distancias duplicadas para que la tabla sea función.
        List<Double> mergedDistances = new ArrayList<>();
        List<Double> mergedRpms = new ArrayList<>();
        int i = 0;
        while (i < sorted.size()) {
            double distance = sorted.get(i).distanceInches;
            double sum = 0;
            int count = 0;
            while (i < sorted.size() && sorted.get(i).distanceInches == distance) {
                sum += sorted.get(i).rpm;
                count++;
                i++;
            }
            mergedDistances.add(distance);
            mergedRpms.add(sum / count);
        }

        double[] distances = new double[mergedDistances.size()];
        double[] rpms = new double[mergedRpms.size()];
        for (int k = 0; k < distances.length; k++) {
            distances[k] = mergedDistances.get(k);
            rpms[k] = mergedRpms.get(k);
        }
        return new PiecewiseLinearRpmModel(distances, rpms);
    }

    @Override
    public double rpmForDistance(double distanceInches) {
        // Adaptado de HyperionBots (FTC 18011): clamp en extremos, interpolación adentro.
        if (distanceInches <= distances[0]) {
            return RpmModel.clampRpm(rpms[0]);
        }
        int last = distances.length - 1;
        if (distanceInches >= distances[last]) {
            return RpmModel.clampRpm(rpms[last]);
        }
        for (int j = 0; j < last; j++) {
            if (distanceInches >= distances[j] && distanceInches <= distances[j + 1]) {
                double t = (distanceInches - distances[j]) / (distances[j + 1] - distances[j]);
                return RpmModel.clampRpm(rpms[j] + t * (rpms[j + 1] - rpms[j]));
            }
        }
        // Inalcanzable por construcción; fail-closed por si acaso.
        return RpmModel.clampRpm(rpms[last]);
    }

    public int pointCount() {
        return distances.length;
    }

    @Override
    public String describe() {
        return String.format(Locale.US, "piecewise(points=%d, d=[%.1f..%.1f])",
                distances.length, distances[0], distances[distances.length - 1]);
    }
}
