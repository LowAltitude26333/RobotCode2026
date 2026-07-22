package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.commands.drivetrain.FieldCentricDriveCommand;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public final class FieldCentricDriveCommandTest {
    private static final double EPS = 1e-9;

    @Test
    public void zeroHeadingKeepsDriverAxes() {
        assertInput(0.0, 1.0,
                FieldCentricDriveCommand.fieldToRobot(0.0, 1.0, 0.0));
        assertInput(1.0, 0.0,
                FieldCentricDriveCommand.fieldToRobot(1.0, 0.0, 0.0));
    }

    @Test
    public void clockwiseNinetyConvertsFieldForwardToRobotLeft() {
        assertInput(-1.0, 0.0,
                FieldCentricDriveCommand.fieldToRobot(
                        0.0, 1.0, -Math.PI / 2.0));
    }

    @Test
    public void clockwiseNinetyConvertsFieldRightToRobotForward() {
        assertInput(0.0, 1.0,
                FieldCentricDriveCommand.fieldToRobot(
                        1.0, 0.0, -Math.PI / 2.0));
    }

    @Test
    public void counterclockwiseNinetyConvertsFieldForwardToRobotRight() {
        assertInput(1.0, 0.0,
                FieldCentricDriveCommand.fieldToRobot(
                        0.0, 1.0, Math.PI / 2.0));
    }

    @Test
    public void driverTurnUsesExpectedDirectionAndLeavesTranslationMargin() {
        assertEquals(-0.70, FieldCentricDriveCommand.shapeDriverTurn(1.0), EPS);
        assertEquals(0.70, FieldCentricDriveCommand.shapeDriverTurn(-1.0), EPS);
        assertEquals(0.0, FieldCentricDriveCommand.shapeDriverTurn(0.0), EPS);
    }

    private static void assertInput(double expectedStrafeRight, double expectedForward,
                                    double[] actual) {
        assertEquals(expectedStrafeRight, actual[0], EPS);
        assertEquals(expectedForward, actual[1], EPS);
    }
}
