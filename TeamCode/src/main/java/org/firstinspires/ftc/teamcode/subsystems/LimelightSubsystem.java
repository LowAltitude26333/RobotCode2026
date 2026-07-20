package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.RobotSafety;
import org.firstinspires.ftc.teamcode.vision.LimelightObservation;
import org.firstinspires.ftc.teamcode.vision.LimelightRawSample;

import java.util.List;

/**
 * Único dueño de la Limelight 3A (MP-03). Nadie más debe mapear el dispositivo.
 *
 * Lifecycle basado en el sample oficial SensorLimelight3A: pipelineSwitch ->
 * start -> getLatestResult (null hasta llamar start) -> stop. El dispositivo es
 * opcional (DEC-028: la Limelight aún no está configurada en el robot), por lo
 * que se usa hardwareMap.tryGet y el subsystem corre degradado sin lanzar.
 * Este subsystem solo publica observaciones inmutables; nunca comanda actuadores.
 */
public class LimelightSubsystem extends SubsystemBase {

    public enum Health { NOT_PRESENT, CREATED, RUNNING, STOPPED }

    private final Limelight3A limelight;
    private int expectedTagId;
    private Health health;
    private volatile LimelightObservation latest;
    private volatile LimelightRawSample latestRaw;

    public LimelightSubsystem(HardwareMap hardwareMap, int expectedTagId) {
        this.expectedTagId = expectedTagId;
        this.limelight = hardwareMap.tryGet(Limelight3A.class, RobotMap.LIMELIGHT);

        if (limelight == null) {
            health = Health.NOT_PRESENT;
            latestRaw = LimelightRawSample.unavailable(false, System.nanoTime());
            latest = classifyRaw(latestRaw);
            RobotLog.addGlobalWarningMessage(
                    RobotMap.LIMELIGHT + " not found; LimelightSubsystem runs degraded (DEC-028)");
        } else {
            limelight.setPollRateHz((int) LowAltitudeConstants.VisionConstants.LIMELIGHT_POLL_RATE_HZ);
            health = Health.CREATED;
            latestRaw = LimelightRawSample.unavailable(true, System.nanoTime());
            latest = classifyRaw(latestRaw);
        }
        RobotSafety.registerShutdown(this::stop);
    }

    public boolean isPresent() {
        return limelight != null;
    }

    public Health getHealth() {
        return health;
    }

    public int getExpectedTagId() {
        return expectedTagId;
    }

    /** Cambiar solo con el OpMode detenido o durante INIT (selección de alianza). */
    public void setExpectedTagId(int tagId) {
        this.expectedTagId = tagId;
    }

    /** Arranca el polling en el pipeline de AprilTags. No-op si el dispositivo está ausente. */
    public void start() {
        if (limelight == null) {
            return;
        }
        limelight.pipelineSwitch(LowAltitudeConstants.VisionConstants.LIMELIGHT_PIPELINE_APRILTAG);
        limelight.start();
        health = Health.RUNNING;
    }

    /** No-op si el dispositivo está ausente. */
    public void setPipeline(int index) {
        if (limelight == null) {
            return;
        }
        limelight.pipelineSwitch(index);
    }

    /** Nunca devuelve null; arranca rechazada y solo periodic() publica VALID. */
    public LimelightObservation getLatestObservation() {
        return latest;
    }

    /** Raw diagnostic channel; never use it for actuator or pose decisions. */
    public LimelightRawSample getLatestRawSample() {
        return latestRaw;
    }

    /** Idempotente y sin lanzar: es hook de RobotSafety y de cleanup del OpMode. */
    public void stop() {
        if (limelight != null) {
            try {
                limelight.stop();
            } catch (RuntimeException ignored) {
                // El hook de safety debe completar aunque el dispositivo falle al detenerse.
            }
            health = Health.STOPPED;
        }
        latestRaw = LimelightRawSample.unavailable(limelight != null, System.nanoTime());
        latest = classifyRaw(latestRaw);
    }

    @Override
    public void periodic() {
        latest = pollObservation();
    }

    private LimelightObservation pollObservation() {
        long now = System.nanoTime();
        if (limelight == null) {
            latestRaw = LimelightRawSample.unavailable(false, now);
            return classifyRaw(latestRaw);
        }
        if (health != Health.RUNNING) {
            latestRaw = LimelightRawSample.unavailable(true, now);
            return classifyRaw(latestRaw);
        }

        // Guard fail-closed de resultado nulo/inválido (patrón HyperionBots FTC 18011,
        // LimeLightGoalTrackingTuning): sin resultado utilizable no se publica nada VALID.
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            latestRaw = LimelightRawSample.unavailable(true, now);
            return classifyRaw(latestRaw);
        }

        int detectedTagId = -1;
        LLResultTypes.FiducialResult selected = null;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials != null && !fiducials.isEmpty()) {
            selected = fiducials.get(0);
            detectedTagId = selected.getFiducialId();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == expectedTagId) {
                    selected = fiducial;
                    detectedTagId = expectedTagId;
                    break;
                }
            }
        }

        double stalenessMs = result.getStaleness();
        double totalLatencyMs = result.getCaptureLatency()
                + result.getTargetingLatency()
                + result.getParseLatency();

        Pose3D botpose = result.getBotpose();
        boolean hasBotPose = botpose != null;
        double botX = 0.0;
        double botY = 0.0;
        double botHeading = 0.0;
        if (hasBotPose) {
            botX = botpose.getPosition().x;
            botY = botpose.getPosition().y;
            botHeading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
        }

        latestRaw = new LimelightRawSample(true, true, now, detectedTagId,
                selected == null ? 0.0 : selected.getTargetXDegrees(),
                selected == null ? 0.0 : selected.getTargetYDegrees(),
                stalenessMs, totalLatencyMs, hasBotPose, botX, botY, botHeading);
        return classifyRaw(latestRaw);
    }

    private LimelightObservation classifyRaw(LimelightRawSample raw) {
        return LimelightObservation.fromRaw(raw,
                LowAltitudeConstants.VisionConstants.LIMELIGHT_MAX_STALENESS_MS,
                expectedTagId);
    }
}
