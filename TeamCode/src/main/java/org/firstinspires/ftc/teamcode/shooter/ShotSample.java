package org.firstinspires.ftc.teamcode.shooter;

/**
 * Punto de ajuste matemático (Tuning Paso 5): distancia al goal y RPM de
 * calibración. No representa intentos ni certifica el gate físico DEC-012;
 * {@link ShotTrial} conserva esos resultados por sesión/distancia.
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
