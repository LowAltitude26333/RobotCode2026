package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LowAltitudeConstants;

/**
 * Compatibility shim for disabled legacy commands. The physical hood was removed and this
 * class intentionally owns no hardware. Active OpModes must not instantiate or command it.
 */
@Deprecated
public class ShooterHoodSubsystem extends SubsystemBase {
    private final Telemetry telemetry;
    private double lastRequestedDegrees;

    public ShooterHoodSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void setPosition(LowAltitudeConstants.HoodPosition position) {
        setAngle(position.angle);
    }

    public void setAngle(double degrees) {
        // Clamp de seguridad con tus nuevos límites (25 a 55)
        double safeAngle = Math.max(
                LowAltitudeConstants.HOOD_MIN_LIMIT,
                Math.min(degrees, LowAltitudeConstants.HOOD_MAX_LIMIT)
        );

        lastRequestedDegrees = safeAngle;
    }

    // Método para debug: Mover por porcentaje (0.0 a 1.0) para encontrar los ángulos reales
    public void setRawPosition(double position) {
        lastRequestedDegrees = position;
    }

    @Override
    public void periodic() {
        // Imprimimos la posición lógica (Grados) y la cruda (0-1)
        telemetry.addData("Hood", "REMOVED (legacy request %.1f ignored)", lastRequestedDegrees);
    }
}
