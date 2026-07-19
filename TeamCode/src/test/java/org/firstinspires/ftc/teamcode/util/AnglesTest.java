package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class AnglesTest {

    private static final double EPS = 1e-12;

    @Test
    public void normalizeRadiansIdentityInsideRange() {
        assertEquals(0.0, Angles.normalizeRadians(0.0), EPS);
        assertEquals(Math.PI / 2, Angles.normalizeRadians(Math.PI / 2), EPS);
        assertEquals(-Math.PI / 2, Angles.normalizeRadians(-Math.PI / 2), EPS);
    }

    @Test
    public void normalizeRadiansBoundarySemantics() {
        // Semántica heredada de DriveSubsystem: rango (-PI, PI].
        assertEquals(Math.PI, Angles.normalizeRadians(Math.PI), EPS);
        assertEquals(Math.PI, Angles.normalizeRadians(-Math.PI), EPS);
    }

    @Test
    public void normalizeRadiansWrapsMultipleTurns() {
        assertEquals(Math.PI, Angles.normalizeRadians(3 * Math.PI), EPS);
        assertEquals(Math.PI, Angles.normalizeRadians(-3 * Math.PI), EPS);
        assertEquals(0.5, Angles.normalizeRadians(10 * Math.PI + 0.5), 1e-9);
        assertEquals(-0.5, Angles.normalizeRadians(-10 * Math.PI - 0.5), 1e-9);
    }

    @Test
    public void normalizeDegreesMatchesRadiansSemantics() {
        assertEquals(180.0, Angles.normalizeDegrees(180.0), 1e-9);
        assertEquals(180.0, Angles.normalizeDegrees(-180.0), 1e-9);
        assertEquals(-90.0, Angles.normalizeDegrees(270.0), 1e-9);
        assertEquals(10.0, Angles.normalizeDegrees(730.0), 1e-9);
    }

    @Test
    public void shortestDeltaBasic() {
        assertEquals(0.5, Angles.shortestDelta(0.0, 0.5), EPS);
        assertEquals(-0.5, Angles.shortestDelta(0.5, 0.0), EPS);
    }

    @Test
    public void shortestDeltaCrossesPiBoundary() {
        // De 170° a -170°: el camino corto son +20°, no -340°.
        double from = Math.toRadians(170.0);
        double to = Math.toRadians(-170.0);
        assertEquals(Math.toRadians(20.0), Angles.shortestDelta(from, to), 1e-9);
        assertEquals(Math.toRadians(-20.0), Angles.shortestDelta(to, from), 1e-9);
    }

    @Test
    public void shortestDeltaOppositeIsHalfTurn() {
        // Ambigüedad de 180°: la semántica (-PI, PI] fija el resultado en +PI.
        assertEquals(Math.PI, Angles.shortestDelta(0.0, Math.PI), EPS);
        assertEquals(Math.PI, Angles.shortestDelta(Math.PI / 2, -Math.PI / 2), EPS);
    }
}
