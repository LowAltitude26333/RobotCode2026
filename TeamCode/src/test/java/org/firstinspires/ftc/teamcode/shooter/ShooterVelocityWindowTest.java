package org.firstinspires.ftc.teamcode.shooter;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class ShooterVelocityWindowTest {

    @Test
    public void calculatesShooterRpmFromPositionDelta() {
        ShooterVelocityWindow window = new ShooterVelocityWindow(100);

        assertEquals(0.0, window.update(0, 0, 28.0, 1.0), 1e-9);
        assertEquals(60.0,
                window.update(28, 1_000_000_000L, 28.0, 1.0), 1e-9);
    }

    @Test
    public void holdsPreviousEstimateUntilWindowCompletes() {
        ShooterVelocityWindow window = new ShooterVelocityWindow(100);

        window.update(0, 0, 28.0, 1.0);
        assertEquals(0.0,
                window.update(14, 50_000_000L, 28.0, 1.0), 1e-9);
        assertEquals(600.0,
                window.update(28, 100_000_000L, 28.0, 1.0), 1e-9);
    }

    @Test
    public void preservesOutwardNegativeMotionForDiagnosis() {
        ShooterVelocityWindow window = new ShooterVelocityWindow(100);

        window.update(0, 0, 28.0, 1.0);
        assertEquals(-600.0,
                window.update(-28, 100_000_000L, 28.0, 1.0), 1e-9);
    }
}
