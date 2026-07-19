package org.firstinspires.ftc.teamcode.shooter;

import static org.firstinspires.ftc.teamcode.shooter.RpmModelsTest.samples;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Test;

import java.util.List;

public class RpmModelSelectorTest {

    private static final double TOLERANCE_RPM = 100.0;

    @Test
    public void exactLinearDataSelectsLinear() {
        // rpm = 25*d + 1800 en calibración Y holdout.
        List<ShotSample> calibration = samples(
                new double[]{20, 35, 50, 65, 80, 95, 110, 125, 140, 155},
                linear(new double[]{20, 35, 50, 65, 80, 95, 110, 125, 140, 155}));
        List<ShotSample> holdout = samples(
                new double[]{27, 42, 58, 73, 88, 103, 118, 133, 148, 152},
                linear(new double[]{27, 42, 58, 73, 88, 103, 118, 133, 148, 152}));

        RpmModelSelector.Result result =
                RpmModelSelector.select(calibration, holdout, TOLERANCE_RPM);
        assertTrue(result.metCriteria);
        assertTrue(result.model instanceof LinearRpmModel);
        assertEquals(1.0, result.calibrationHitRate, 0.0);
        assertEquals(1.0, result.holdoutHitRate, 0.0);
    }

    @Test
    public void strongCurvatureSelectsQuadratic() {
        // rpm = 0.35*d^2 + 1500: la curvatura excede ±100 RPM para un ajuste
        // lineal, y el máximo (d=100 -> 5000 RPM) queda bajo el clamp de 6000.
        double[] calDistances = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
        double[] holdDistances = {15, 25, 35, 45, 55, 65, 75, 85, 95, 98};
        List<ShotSample> calibration = samples(calDistances, quadratic(calDistances));
        List<ShotSample> holdout = samples(holdDistances, quadratic(holdDistances));

        RpmModelSelector.Result result =
                RpmModelSelector.select(calibration, holdout, TOLERANCE_RPM);
        assertTrue(result.metCriteria);
        assertTrue("esperaba cuadrático, fue " + result.model.describe(),
                result.model instanceof QuadraticRpmModel);
    }

    @Test
    public void kneeShapedDataSelectsPiecewise() {
        // Rodilla: plano hasta 80 in y pendiente fuerte después — ni línea ni
        // parábola de grado 2 la siguen dentro de ±100 RPM.
        double[] calDistances = {20, 35, 50, 65, 80, 90, 100, 110, 120, 130};
        double[] holdDistances = {28, 44, 58, 72, 84, 94, 104, 114, 124, 128};
        List<ShotSample> calibration = samples(calDistances, knee(calDistances));
        List<ShotSample> holdout = samples(holdDistances, knee(holdDistances));

        RpmModelSelector.Result result =
                RpmModelSelector.select(calibration, holdout, TOLERANCE_RPM);
        assertTrue(result.metCriteria);
        assertTrue("esperaba piecewise, fue " + result.model.describe(),
                result.model instanceof PiecewiseLinearRpmModel);
    }

    @Test
    public void noisyHoldoutFailsVisibly() {
        // Calibración limpia pero holdout incoherente (>20% fuera de tolerancia
        // para cualquier modelo): debe devolver NO_MODEL_MET_CRITERIA, no ocultarlo.
        List<ShotSample> calibration = samples(
                new double[]{20, 40, 60, 80, 100},
                linear(new double[]{20, 40, 60, 80, 100}));
        List<ShotSample> holdout = samples(
                new double[]{25, 45, 65, 85, 105},
                new double[]{9000, 100, 9000, 100, 9000});

        RpmModelSelector.Result result =
                RpmModelSelector.select(calibration, holdout, TOLERANCE_RPM);
        assertFalse(result.metCriteria);
        assertTrue(result.rationale.contains("NO_MODEL_MET_CRITERIA"));
        assertTrue(result.model instanceof PiecewiseLinearRpmModel);
    }

    @Test
    public void invalidInputsRejected() {
        List<ShotSample> some = samples(new double[]{10, 20}, new double[]{2000, 2200});
        try {
            RpmModelSelector.select(null, some, TOLERANCE_RPM);
            fail("calibración null debía fallar");
        } catch (IllegalArgumentException expected) {
        }
        try {
            RpmModelSelector.select(some, some, 0.0);
            fail("tolerancia 0 debía fallar");
        } catch (IllegalArgumentException expected) {
        }
    }

    private static double[] linear(double[] distances) {
        double[] rpms = new double[distances.length];
        for (int i = 0; i < distances.length; i++) {
            rpms[i] = 25 * distances[i] + 1800;
        }
        return rpms;
    }

    private static double[] quadratic(double[] distances) {
        double[] rpms = new double[distances.length];
        for (int i = 0; i < distances.length; i++) {
            rpms[i] = 0.35 * distances[i] * distances[i] + 1500;
        }
        return rpms;
    }

    private static double[] knee(double[] distances) {
        double[] rpms = new double[distances.length];
        for (int i = 0; i < distances.length; i++) {
            double d = distances[i];
            rpms[i] = d <= 80 ? 2000 + 2 * d : 2160 + 45 * (d - 80);
        }
        return rpms;
    }
}
