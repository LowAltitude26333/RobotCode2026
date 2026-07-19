package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.opmodes.SafeCommandOpMode;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.vision.LimelightObservation;

/**
 * Pantalla de commissioning de la Limelight (Tuning Paso 4: montaje, red, anclas).
 * Cero actuadores: solo construye LimelightSubsystem y muestra la observación
 * cruda con su veredicto de calidad. Alianza con dpad durante INIT.
 */
@TeleOp(name = "Limelight Diagnostic", group = "Diagnostics")
public class LimelightDiagnosticOpMode extends SafeCommandOpMode {

    private LimelightSubsystem limelightSubsystem;
    private boolean allianceRed = false;

    @Override
    public void initialize() {
        limelightSubsystem = new LimelightSubsystem(hardwareMap,
                LowAltitudeConstants.TurretConstants.BLUE_GOAL_TAG_ID);
        // FND-001: registrar el cleanup DESPUÉS de construir el recurso, nunca antes.
        addResourceCleanup(limelightSubsystem::stop);
        // Polling de cámara, sin actuadores: arrancar desde INIT para que Tuning
        // pueda alinear montaje/anclas antes de START.
        limelightSubsystem.start();
    }

    @Override
    protected void duringInitLoop() {
        if (gamepad1.dpad_left) {
            allianceRed = false;
        } else if (gamepad1.dpad_right) {
            allianceRed = true;
        }
        limelightSubsystem.setExpectedTagId(allianceRed
                ? LowAltitudeConstants.TurretConstants.RED_GOAL_TAG_ID
                : LowAltitudeConstants.TurretConstants.BLUE_GOAL_TAG_ID);

        // El scheduler no corre durante INIT; poll manual para dar datos en vivo.
        limelightSubsystem.periodic();

        telemetry.addLine("=== LIMELIGHT DIAGNOSTIC (INIT) ===");
        telemetry.addLine("Dpad IZQ = BLUE | Dpad DER = RED");
        addLimelightTelemetry();
    }

    @Override
    public void run() {
        super.run();
        telemetry.addLine("=== LIMELIGHT DIAGNOSTIC ===");
        addLimelightTelemetry();
        telemetry.update();
    }

    private void addLimelightTelemetry() {
        LimelightObservation obs = limelightSubsystem.getLatestObservation();
        telemetry.addData("Alianza", allianceRed ? "RED" : "BLUE");
        telemetry.addData("Tag esperado", limelightSubsystem.getExpectedTagId());
        telemetry.addData("LL/Presente", limelightSubsystem.isPresent());
        telemetry.addData("LL/Health", limelightSubsystem.getHealth());
        telemetry.addData("LL/Quality", obs.quality);
        telemetry.addData("LL/Usable", obs.isUsable());
        telemetry.addData("LL/Tag", obs.tagId);
        telemetry.addData("LL/tx", "%.2f°", obs.txDegrees);
        telemetry.addData("LL/ty", "%.2f°", obs.tyDegrees);
        telemetry.addData("LL/Staleness", "%.1f ms", obs.stalenessMs);
        telemetry.addData("LL/Latencia total", "%.1f ms", obs.totalLatencyMs);
        telemetry.addData("LL/Edad obs", "%.1f ms",
                (System.nanoTime() - obs.timestampNanos) / 1_000_000.0);
        if (obs.hasBotPose) {
            telemetry.addData("LL/Botpose", "X %.2f m, Y %.2f m, H %.1f°",
                    obs.botPoseXMeters, obs.botPoseYMeters,
                    Math.toDegrees(obs.botPoseHeadingRadians));
        } else {
            telemetry.addData("LL/Botpose", "NO DISPONIBLE");
        }
    }
}
