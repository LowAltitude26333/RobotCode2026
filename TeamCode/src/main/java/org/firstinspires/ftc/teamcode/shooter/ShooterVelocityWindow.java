package org.firstinspires.ftc.teamcode.shooter;

/**
 * Independent RPM estimate from outward-positive encoder position deltas.
 *
 * <p>This intentionally does not depend on the SDK/FTCLib velocity estimate so
 * commissioning can compare both measurement paths without changing control.</p>
 */
public final class ShooterVelocityWindow {

    private final long minimumWindowNanos;
    private boolean initialized;
    private int startTicks;
    private long startNanos;
    private double rpm;

    public ShooterVelocityWindow(long minimumWindowMillis) {
        if (minimumWindowMillis <= 0) {
            throw new IllegalArgumentException("minimumWindowMillis must be positive");
        }
        minimumWindowNanos = minimumWindowMillis * 1_000_000L;
    }

    public double update(int outwardTicks, long nowNanos,
                         double motorTicksPerRevolution, double shooterGearRatio) {
        if (!Double.isFinite(motorTicksPerRevolution)
                || motorTicksPerRevolution <= 0
                || !Double.isFinite(shooterGearRatio)
                || shooterGearRatio <= 0) {
            throw new IllegalArgumentException("ticks/rev and gear ratio must be positive");
        }

        if (!initialized || nowNanos < startNanos) {
            initialized = true;
            startTicks = outwardTicks;
            startNanos = nowNanos;
            rpm = 0;
            return rpm;
        }

        long elapsedNanos = nowNanos - startNanos;
        if (elapsedNanos < minimumWindowNanos) {
            return rpm;
        }

        double elapsedSeconds = elapsedNanos / 1_000_000_000.0;
        double motorRevolutions = (outwardTicks - startTicks) / motorTicksPerRevolution;
        rpm = motorRevolutions * 60.0 / elapsedSeconds * shooterGearRatio;
        startTicks = outwardTicks;
        startNanos = nowNanos;
        return rpm;
    }
}
