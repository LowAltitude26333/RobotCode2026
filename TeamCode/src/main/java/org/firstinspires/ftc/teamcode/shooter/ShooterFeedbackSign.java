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

    /**
     * FTC SDK motor direction changes the sign returned by its encoder reads.
     * Apply only the remaining inversion needed after the motor direction is configured.
     */
    public static boolean afterMotorDirection(boolean rawEncoderIsInverted,
                                              boolean motorIsInverted) {
        return rawEncoderIsInverted != motorIsInverted;
    }
}
