package org.firstinspires.ftc.teamcode.localization;

import static org.junit.Assert.assertArrayEquals;

import org.junit.Test;

public class PedroDriveAdapterTest {
    private static final double EPS = 1e-9;

    @Test
    public void neutralDriveSignsMapToPedro() {
        assertArrayEquals(new double[]{0.4, -0.3, 0.2},
                PedroDriveAdapter.toPedroRobotCentric(0.3, 0.4, 0.2), EPS);
    }

    @Test
    public void driveCommandsClampAndNonFiniteFailsClosed() {
        assertArrayEquals(new double[]{1.0, 1.0, -1.0},
                PedroDriveAdapter.toPedroRobotCentric(-2.0, 3.0, -4.0), EPS);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0},
                PedroDriveAdapter.toPedroRobotCentric(Double.NaN, 0.5, 0.5), EPS);
    }

    @Test
    public void fieldVelocityRotatesIntoRobotFrame() {
        assertArrayEquals(new double[]{0.0, -10.0},
                PedroDriveAdapter.fieldVelocityToRobot(10.0, 0.0, Math.PI / 2.0), EPS);
        assertArrayEquals(new double[]{4.0, -2.0},
                PedroDriveAdapter.fieldVelocityToRobot(4.0, -2.0, 0.0), EPS);
    }
}
