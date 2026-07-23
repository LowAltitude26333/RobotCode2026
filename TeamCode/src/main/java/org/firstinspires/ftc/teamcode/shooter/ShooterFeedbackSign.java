package org.firstinspires.ftc.teamcode.shooter;

/**
 * Defines the shooter feedback convention independently from motor-power direction.
 * Positive values mean the physically verified outward shooting direction.
 */
public final class ShooterFeedbackSign {

    private ShooterFeedbackSign() {
    }

    public static double outwardPositive(double rawValue, boolean encoderIsInverted) {
        return encoderIsInverted ? -rawValue : rawValue;
    }

    public static int outwardPositive(int rawValue, boolean encoderIsInverted) {
        return encoderIsInverted ? -rawValue : rawValue;
    }
}
