package org.firstinspires.ftc.teamcode.shooter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class RpmModelsTest {

    @Test
    public void linearFitRecoversExactLine() {
        // rpm = 20*d + 2000
        LinearRpmModel model = LinearRpmModel.fit(samples(
                new double[]{20, 40, 60, 80}, new double[]{2400, 2800, 3200, 3600}));
        assertEquals(2400, model.rpmForDistance(20), 1e-6);
        assertEquals(3000, model.rpmForDistance(50), 1e-6);
        assertEquals(3600, model.rpmForDistance(80), 1e-6);
    }

    @Test
    public void quadraticFitRecoversExactParabola() {
        // rpm = 0.3*d^2 + 5*d + 1500 (máximo en d=90 -> 4380, bajo el clamp de 6000)
        double[] distances = {10, 30, 50, 70, 90};
        double[] rpms = new double[distances.length];
        for (int i = 0; i < distances.length; i++) {
            rpms[i] = 0.3 * distances[i] * distances[i] + 5 * distances[i] + 1500;
        }
        QuadraticRpmModel model = QuadraticRpmModel.fit(samples(distances, rpms));
        assertEquals(0.3 * 40 * 40 + 5 * 40 + 1500, model.rpmForDistance(40), 1e-4);
        assertEquals(0.3 * 90 * 90 + 5 * 90 + 1500, model.rpmForDistance(90), 1e-4);
    }

    @Test
    public void piecewiseInterpolatesBetweenPoints() {
        PiecewiseLinearRpmModel model = PiecewiseLinearRpmModel.of(samples(
                new double[]{44, 80, 105}, new double[]{1065, 1180, 1320}));
        // Interpolación entre 44 y 80: mitad del segmento.
        assertEquals((1065 + 1180) / 2.0, model.rpmForDistance(62), 1e-9);
        // Puntos exactos.
        assertEquals(1180, model.rpmForDistance(80), 1e-9);
    }

    @Test
    public void piecewiseSaturatesOutOfRange() {
        PiecewiseLinearRpmModel model = PiecewiseLinearRpmModel.of(samples(
                new double[]{44, 80, 105}, new double[]{1065, 1180, 1320}));
        // Patrón HyperionBots: clamp a los extremos, sin extrapolar.
        assertEquals(1065, model.rpmForDistance(0), 1e-9);
        assertEquals(1065, model.rpmForDistance(44), 1e-9);
        assertEquals(1320, model.rpmForDistance(105), 1e-9);
        assertEquals(1320, model.rpmForDistance(500), 1e-9);
    }

    @Test
    public void piecewiseAveragesDuplicateDistances() {
        PiecewiseLinearRpmModel model = PiecewiseLinearRpmModel.of(samples(
                new double[]{50, 50, 100}, new double[]{2000, 2200, 3000}));
        assertEquals(2, model.pointCount());
        assertEquals(2100, model.rpmForDistance(50), 1e-9);
    }

    @Test
    public void allModelsClampToMaxSafeRpm() {
        // Línea que extrapola muy por encima de 6000 RPM.
        LinearRpmModel linear = LinearRpmModel.fit(samples(
                new double[]{10, 20}, new double[]{3000, 5000}));
        assertTrue(linear.rpmForDistance(1000) <= 6000.0);
        assertTrue(linear.rpmForDistance(-1000) >= 0.0);

        QuadraticRpmModel quadratic = QuadraticRpmModel.fit(samples(
                new double[]{10, 20, 30}, new double[]{3000, 4000, 5500}));
        assertTrue(quadratic.rpmForDistance(500) <= 6000.0);

        PiecewiseLinearRpmModel piecewise = PiecewiseLinearRpmModel.of(samples(
                new double[]{10}, new double[]{9999}));
        assertEquals(6000.0, piecewise.rpmForDistance(10), 1e-9);
    }

    @Test
    public void allModelsReturnZeroForInvalidDistance() {
        LinearRpmModel linear = LinearRpmModel.fit(samples(
                new double[]{10, 20}, new double[]{3000, 3500}));
        QuadraticRpmModel quadratic = QuadraticRpmModel.fit(samples(
                new double[]{10, 20, 30}, new double[]{3000, 3500, 4200}));
        PiecewiseLinearRpmModel piecewise = PiecewiseLinearRpmModel.of(samples(
                new double[]{10, 20}, new double[]{3000, 3500}));

        RpmModel[] models = {linear, quadratic, piecewise};
        double[] invalidDistances = {
                -1.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY
        };
        for (RpmModel model : models) {
            for (double distance : invalidDistances) {
                assertEquals(0.0, model.rpmForDistance(distance), 0.0);
            }
        }
    }

    @Test
    public void degenerateDatasetsRejectedWithClearErrors() {
        try {
            LinearRpmModel.fit(Collections.<ShotSample>emptyList());
            fail("lineal con lista vacía debía fallar");
        } catch (IllegalArgumentException expected) {
        }
        try {
            LinearRpmModel.fit(samples(new double[]{50, 50}, new double[]{2000, 2100}));
            fail("lineal con una sola distancia distinta debía fallar");
        } catch (IllegalArgumentException expected) {
        }
        try {
            QuadraticRpmModel.fit(samples(new double[]{10, 20}, new double[]{2000, 2100}));
            fail("cuadrático con 2 distancias distintas debía fallar");
        } catch (IllegalArgumentException expected) {
        }
        try {
            PiecewiseLinearRpmModel.of(new ArrayList<ShotSample>());
            fail("piecewise con lista vacía debía fallar");
        } catch (IllegalArgumentException expected) {
        }
        try {
            new ShotSample(-1, 2000);
            fail("ShotSample negativo debía fallar");
        } catch (IllegalArgumentException expected) {
        }
        try {
            new ShotSample(10, Double.NaN);
            fail("ShotSample NaN debía fallar");
        } catch (IllegalArgumentException expected) {
        }
        try {
            LinearRpmModel.fit(samples(
                    new double[]{1, Double.MAX_VALUE},
                    new double[]{1, Double.MAX_VALUE}));
            fail("overflow lineal debía fallar");
        } catch (IllegalArgumentException expected) {
        }
        try {
            QuadraticRpmModel.fit(samples(
                    new double[]{1, 2, Double.MAX_VALUE},
                    new double[]{1, 2, Double.MAX_VALUE}));
            fail("overflow cuadrático debía fallar");
        } catch (IllegalArgumentException expected) {
        }
        try {
            PiecewiseLinearRpmModel.of(samples(
                    new double[]{10, 10},
                    new double[]{Double.MAX_VALUE, Double.MAX_VALUE}));
            fail("overflow piecewise debía fallar");
        } catch (IllegalArgumentException expected) {
        }
    }

    static List<ShotSample> samples(double[] distances, double[] rpms) {
        List<ShotSample> list = new ArrayList<>();
        for (int i = 0; i < distances.length; i++) {
            list.add(new ShotSample(distances[i], rpms[i]));
        }
        return list;
    }

}
