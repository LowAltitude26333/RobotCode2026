package org.firstinspires.ftc.teamcode.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.localization.PoseSnapshot;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.vision.LimelightObservation;
import org.firstinspires.ftc.teamcode.vision.LimelightRawSample;

/**
 * Bloques de telemetría estandarizados (prep de MP-07). Helper estático sin
 * estado: solo formatea, nunca toca actuadores ni el scheduler. MP-07 migrará
 * los addData dispersos de los OpModes a estos bloques
 * (ver docs/plan-maestro/inventario-telemetria-tuners.md).
 */
public final class TelemetryBlocks {

    private TelemetryBlocks() {
    }

    public static void header(Telemetry t, String name) {
        t.addLine("=== " + name + " ===");
    }

    public static void mode(Telemetry t, boolean isRed, boolean isPrecision) {
        header(t, "MODO");
        t.addData("Alianza", isRed ? "RED" : "BLUE");
        t.addData("Precisión", isPrecision);
    }

    public static void pose(Telemetry t, PoseSnapshot snapshot) {
        header(t, "POSE");
        t.addData("Pose/Quality", snapshot.quality);
        t.addData("Pose/Usable", snapshot.isUsable());
        if (!snapshot.rejectionReason.isEmpty()) {
            t.addData("Pose/Rechazo", snapshot.rejectionReason);
        }
        t.addData("Pose", "X %.1f in, Y %.1f in, H %.1f°",
                snapshot.xInches, snapshot.yInches, Math.toDegrees(snapshot.headingRadians));
        t.addData("Pose/Vel robot", "vx %.1f, vy %.1f, w %.2f",
                snapshot.vxInchesPerSec, snapshot.vyInchesPerSec, snapshot.omegaRadiansPerSec);
        t.addData("Pose/ResetEpoch", snapshot.resetEpoch);
    }

    public static void vision(Telemetry t, LimelightObservation obs,
                              LimelightSubsystem.Health health, int expectedTagId) {
        header(t, "VISIÓN");
        t.addData("LL/Health", health);
        t.addData("LL/Quality", obs.quality);
        t.addData("LL/Usable", obs.isUsable());
        if (!obs.rejectionReason.isEmpty()) {
            t.addData("LL/Rechazo", obs.rejectionReason);
        }
        t.addData("LL/Tag", "%d (esperado %d)", obs.tagId, expectedTagId);
        t.addData("LL/tx-ty", "%.2f°, %.2f°", obs.txDegrees, obs.tyDegrees);
        t.addData("LL/Staleness", "%.1f ms", obs.stalenessMs);
        t.addData("LL/Latencia", "%.1f ms", obs.totalLatencyMs);
        if (obs.hasBotPose) {
            t.addData("LL/Botpose", "X %.2f m, Y %.2f m, H %.1f°",
                    obs.botPoseXMeters, obs.botPoseYMeters,
                    Math.toDegrees(obs.botPoseHeadingRadians));
        } else {
            t.addData("LL/Botpose", "NO DISPONIBLE");
        }
    }

    /** Unsanitized values for the zero-actuator diagnostic OpMode only. */
    public static void visionRaw(Telemetry t, LimelightRawSample raw) {
        header(t, "VISIÓN RAW (NO ACCIONABLE)");
        t.addData("LL Raw/Present-Valid", "%s / %s", raw.devicePresent, raw.resultValid);
        t.addData("LL Raw/Tag", raw.tagId);
        t.addData("LL Raw/tx-ty", "%s°, %s°",
                Double.toString(raw.txDegrees), Double.toString(raw.tyDegrees));
        t.addData("LL Raw/Staleness", "%s ms", Double.toString(raw.stalenessMs));
        t.addData("LL Raw/Latencia", "%s ms", Double.toString(raw.totalLatencyMs));
        if (raw.hasBotPose) {
            t.addData("LL Raw/Botpose", "X %s m, Y %s m, H %s rad",
                    Double.toString(raw.botPoseXMeters),
                    Double.toString(raw.botPoseYMeters),
                    Double.toString(raw.botPoseHeadingRadians));
        } else {
            t.addData("LL Raw/Botpose", "NO DISPONIBLE");
        }
    }

    public static void turret(Telemetry t, double ticks, boolean armed, String state) {
        header(t, "TORRETA");
        t.addData("Turret/Armed", armed);
        t.addData("Turret/Ticks", "%.0f", ticks);
        t.addData("Turret/Estado", state);
    }

    public static void shooter(Telemetry t, double targetRpm, double currentRpm, boolean ready) {
        header(t, "SHOOTER");
        t.addData("Shooter/Target", "%.0f RPM", targetRpm);
        t.addData("Shooter/Actual", "%.0f RPM", currentRpm);
        t.addData("Shooter/Ready", ready);
    }

    public static void faults(Telemetry t, String... activeFaults) {
        header(t, "FAULTS");
        if (activeFaults == null || activeFaults.length == 0) {
            t.addData("Faults", "NINGUNO");
            return;
        }
        for (String fault : activeFaults) {
            t.addData("Fault", fault);
        }
    }
}
