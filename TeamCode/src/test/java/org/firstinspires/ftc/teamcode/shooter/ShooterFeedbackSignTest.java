package org.firstinspires.ftc.teamcode.shooter;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.RobotMap;
import org.junit.Test;

public class ShooterFeedbackSignTest {

    @Test
    public void physicalOutwardRotationIsReportedPositive() {
        // T8.1 physical evidence: correct outward rotation reported 0 -> -170 raw ticks.
        assertEquals(170, ShooterFeedbackSign.outwardPositive(
                -170, RobotMap.SHOOTER_ENCODER_IS_INVERTED));
        assertEquals(1028.6, ShooterFeedbackSign.outwardPositive(
                -1028.6, RobotMap.SHOOTER_ENCODER_IS_INVERTED), 1e-9);
    }

    @Test
    public void nonInvertedFeedbackPreservesItsSign() {
        assertEquals(170, ShooterFeedbackSign.outwardPositive(170, false));
        assertEquals(1028.6, ShooterFeedbackSign.outwardPositive(1028.6, false), 1e-9);
    }

    @Test
    public void invertedMotorDirectionDoesNotDoubleInvertSdkFeedback() {
        boolean sdkFeedbackIsInverted = ShooterFeedbackSign.afterMotorDirection(
                RobotMap.SHOOTER_ENCODER_IS_INVERTED,
                RobotMap.SHOOTER_MOTOR_IS_INVERTED);

        // Physical evidence: after the SDK motor direction was REVERSE, outward
        // motion arrived from the SDK as positive before the old second inversion.
        assertEquals(false, sdkFeedbackIsInverted);
        assertEquals(1454, ShooterFeedbackSign.outwardPositive(
                1454, sdkFeedbackIsInverted));
        assertEquals(1542.9, ShooterFeedbackSign.outwardPositive(
                1542.9, sdkFeedbackIsInverted), 1e-9);
    }

    @Test
    public void rawDiagnosticStillUsesPhysicalEncoderSign() {
        assertEquals(true, ShooterFeedbackSign.afterMotorDirection(true, false));
        assertEquals(280, ShooterFeedbackSign.outwardPositive(-280, true));
    }
}
