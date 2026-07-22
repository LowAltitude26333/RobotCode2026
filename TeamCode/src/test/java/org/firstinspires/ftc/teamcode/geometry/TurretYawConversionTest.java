package org.firstinspires.ftc.teamcode.geometry;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class TurretYawConversionTest {

    private static final double DEGREES_TOLERANCE = 0.01;

    @Test
    public void ticksPerDegreeMatchesDocumentedApproximation() {
        // geometria-robot-mp04.md sección 4.1: "≈ 6.0807 ticks/°".
        assertEquals(6.0807, TurretYawConversion.TICKS_PER_DEGREE, 0.001);
    }

    @Test
    public void zeroTicksIsZeroYaw() {
        assertEquals(0.0, TurretYawConversion.ticksToYawDegrees(0.0), 0.0);
        assertEquals(0.0, TurretYawConversion.yawDegreesToTicks(0.0), 0.0);
    }

    @Test
    public void limitRightMatchesDocumentedWorkedExample() {
        // geometria-robot-mp04.md sección 4.1: LIMIT_RIGHT = +1070 -> yaw ≈ -175.97°.
        double yaw = TurretYawConversion.ticksToYawDegrees(1070);
        assertEquals(-175.97, yaw, DEGREES_TOLERANCE);
    }

    @Test
    public void limitLeftMatchesDocumentedWorkedExample() {
        // geometria-robot-mp04.md sección 4.1: LIMIT_LEFT = -983 -> yaw ≈ +161.66°.
        double yaw = TurretYawConversion.ticksToYawDegrees(-983);
        assertEquals(161.66, yaw, DEGREES_TOLERANCE);
    }

    @Test
    public void positiveTicksSweepToNegativeYaw() {
        // Signo invertido: motor/cable del lado derecho -> ticks positivos = yaw negativo.
        assertEquals(-100.0 / TurretYawConversion.TICKS_PER_DEGREE,
                TurretYawConversion.ticksToYawDegrees(100.0), 1e-9);
    }

    @Test
    public void roundTripConversionRecoversOriginalTicks() {
        double[] sampleTicks = {0, 1070, -983, 250, -400};
        for (double ticks : sampleTicks) {
            double yaw = TurretYawConversion.ticksToYawDegrees(ticks);
            double recovered = TurretYawConversion.yawDegreesToTicks(yaw);
            assertEquals(ticks, recovered, 1e-9);
        }
    }
}
