package org.firstinspires.ftc.teamcode.shooter;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.junit.Test;

public class ShooterHardwareConfigTest {

    @Test
    public void matchesVerifiedYellowJacketAndExternalDrive() {
        assertEquals(28.0, LowAltitudeConstants.MOTOR_TICKS_PER_REV, 0.0);
        assertEquals(6000.0, LowAltitudeConstants.MOTOR_MAX_RPM, 0.0);
        assertEquals(1.0, LowAltitudeConstants.SHOOTER_GEAR_RATIO, 0.0);
    }
}
