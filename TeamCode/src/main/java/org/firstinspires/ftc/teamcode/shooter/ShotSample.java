package org.firstinspires.ftc.teamcode.shooter;

/**
 * Punto medido del dataset de tiro (Tuning Paso 5): distancia al goal y RPM
 * que produjo un tiro exitoso. Inmutable y puro (testeable en src/test).
 */
public final class ShotSample {

    public final double distanceInches;
    public final double rpm;

    public ShotSample(double distanceInches, double rpm) {
        if (!Double.isFinite(distanceInches) || !Double.isFinite(rpm)) {
            throw new IllegalArgumentException(
                    "ShotSample requiere valores finitos: distancia=" + distanceInches + ", rpm=" + rpm);
        }
        if (distanceInches < 0 || rpm < 0) {
            throw new IllegalArgumentException(
                    "ShotSample no admite negativos: distancia=" + distanceInches + ", rpm=" + rpm);
        }
        this.distanceInches = distanceInches;
        this.rpm = rpm;
    }
}
