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
 * Lifecycle basado en el SDK 10.3.0: pipelineSwitch -> start ->
 * getLatestResult/isConnected -> stop. Desde 10.3 getLatestResult devuelve un
 * resultado inválido en vez de null mientras aún no hay datos. El dispositivo es
 * opcional (DEC-028: la Limelight aún no está configurada en el robot), por lo
 * que se usa hardwareMap.tryGet y el subsystem corre degradado sin lanzar.
 * Este subsystem solo publica observaciones inmutables; nunca comanda actuadores.
 */
public class LimelightSubsystem extends SubsystemBase implements AutoCloseable {

    public enum Health {
        NOT_PRESENT,
        CREATED,
        RUNNING,
        DISCONNECTED,
        PIPELINE_ERROR,
        ERROR,
        STOPPED
    }

    private final Limelight3A limelight;
    private int expectedTagId;
    private int requestedPipelineIndex;
    private boolean pipelineSwitchAccepted;
    private boolean configurationValid;
    private volatile Health health;
    private volatile LimelightObservation latest;
    private volatile LimelightRawSample latestRaw;

    public LimelightSubsystem(HardwareMap hardwareMap, int expectedTagId) {
        this.expectedTagId = expectedTagId;
        this.requestedPipelineIndex =
                LowAltitudeConstants.VisionConstants.LIMELIGHT_PIPELINE_APRILTAG;
        this.limelight = hardwareMap.tryGet(Limelight3A.class, RobotMap.LIMELIGHT);

        if (limelight == null) {
            health = Health.NOT_PRESENT;
            configurationValid = false;
            latestRaw = LimelightRawSample.unavailable(false, false, System.nanoTime());
            latest = classifyRaw(latestRaw);
            RobotLog.addGlobalWarningMessage(
                    RobotMap.LIMELIGHT + " not found; LimelightSubsystem runs degraded (DEC-028)");
        } else {
            configurationValid = configurePollRate();
            health = configurationValid ? Health.CREATED : Health.ERROR;
            latestRaw = LimelightRawSample.unavailable(true, false, System.nanoTime());
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

    public int getRequestedPipelineIndex() {
        return requestedPipelineIndex;
    }

    /** Cambiar solo con el OpMode detenido o durante INIT (selección de alianza). */
    public void setExpectedTagId(int tagId) {
        this.expectedTagId = tagId;
        latest = classifyRaw(latestRaw);
    }

    /**
     * Arranca o reintenta el polling. Devuelve false si falta configuración,
     * falla el pipeline o el SDK no deja el executor corriendo.
     */
    public synchronized boolean start() {
        if (limelight == null || !configurationValid) {
            return false;
        }
        try {
            pipelineSwitchAccepted = limelight.pipelineSwitch(requestedPipelineIndex);
            limelight.start();
            if (!limelight.isRunning()) {
                health = Health.ERROR;
                return false;
            }
            health = pipelineSwitchAccepted
                    ? (limelight.isConnected() ? Health.RUNNING : Health.DISCONNECTED)
                    : Health.PIPELINE_ERROR;
            return pipelineSwitchAccepted;
        } catch (RuntimeException exception) {
            health = Health.ERROR;
            publishUnavailableSafely();
            return false;
        }
    }

    /** Selecciona el pipeline esperado; nunca habilita consumidores. */
    public synchronized boolean setPipeline(int index) {
        requestedPipelineIndex = index;
        if (limelight == null || !configurationValid || index < 0) {
            pipelineSwitchAccepted = false;
            health = limelight == null ? Health.NOT_PRESENT : Health.PIPELINE_ERROR;
            latest = classifyRaw(latestRaw);
            return false;
        }
        try {
            pipelineSwitchAccepted = limelight.pipelineSwitch(index);
            if (!pipelineSwitchAccepted) {
                health = Health.PIPELINE_ERROR;
            }
            latest = classifyRaw(latestRaw);
            return pipelineSwitchAccepted;
        } catch (RuntimeException exception) {
            pipelineSwitchAccepted = false;
            health = Health.ERROR;
            publishUnavailableSafely();
            return false;
        }
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
        latestRaw = LimelightRawSample.unavailable(limelight != null, false,
                System.nanoTime());
        latest = classifyRaw(latestRaw);
    }

    @Override
    public void periodic() {
        try {
            latest = pollObservation();
        } catch (RuntimeException exception) {
            health = Health.ERROR;
            publishUnavailableSafely();
        }
    }

    private LimelightObservation pollObservation() {
        long now = System.nanoTime();
        if (limelight == null) {
            latestRaw = LimelightRawSample.unavailable(false, false, now);
            return classifyRaw(latestRaw);
        }
        if (!limelight.isRunning()) {
            latestRaw = LimelightRawSample.unavailable(true, false, now);
            return classifyRaw(latestRaw);
        }

        boolean connected = limelight.isConnected();
        health = pipelineSwitchAccepted
                ? (connected ? Health.RUNNING : Health.DISCONNECTED)
                : Health.PIPELINE_ERROR;

        // Guard fail-closed de resultado nulo/inválido (patrón HyperionBots FTC 18011,
        // LimeLightGoalTrackingTuning): sin resultado utilizable no se publica nada VALID.
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            latestRaw = LimelightRawSample.unavailable(true, connected, now);
            return classifyRaw(latestRaw);
        }

        int detectedTagId = -1;
        String detectedTagFamily = "";
        LLResultTypes.FiducialResult selected = null;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials != null && !fiducials.isEmpty()) {
            selected = fiducials.get(0);
            detectedTagId = selected.getFiducialId();
            detectedTagFamily = selected.getFamily();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == expectedTagId) {
                    selected = fiducial;
                    detectedTagId = expectedTagId;
                    detectedTagFamily = fiducial.getFamily();
                    break;
                }
            }
        }

        double stalenessMs = result.getStaleness();
        double totalLatencyMs = result.getCaptureLatency()
                + result.getTargetingLatency()
                + result.getParseLatency();

        Pose3D botpose = result.getBotpose();
        boolean hasBotPose = botpose != null && result.getBotposeTagCount() > 0;
        double botX = 0.0;
        double botY = 0.0;
        double botHeading = 0.0;
        if (hasBotPose) {
            botX = botpose.getPosition().x;
            botY = botpose.getPosition().y;
            botHeading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
        }

        latestRaw = new LimelightRawSample(true, connected, true, now,
                result.getControlHubTimeStamp(), result.getPipelineIndex(),
                fiducials == null ? 0 : fiducials.size(), detectedTagId,
                detectedTagFamily,
                selected == null ? 0.0 : selected.getTargetXDegrees(),
                selected == null ? 0.0 : selected.getTargetYDegrees(),
                selected == null ? 0.0 : selected.getTargetArea(),
                stalenessMs, totalLatencyMs, hasBotPose, botX, botY, botHeading);
        return classifyRaw(latestRaw);
    }

    private LimelightObservation classifyRaw(LimelightRawSample raw) {
        if (raw != null && raw.devicePresent && raw.deviceConnected
                && raw.resultValid && !pipelineSwitchAccepted) {
            return LimelightObservation.rejected(
                    LimelightObservation.Quality.WRONG_PIPELINE,
                    raw.pollTimestampNanos, "PIPELINE_SWITCH_NOT_CONFIRMED");
        }
        return LimelightObservation.fromRaw(raw,
                LowAltitudeConstants.VisionConstants.LIMELIGHT_MAX_STALENESS_MS,
                expectedTagId, requestedPipelineIndex);
    }

    private boolean configurePollRate() {
        double requestedRate = LowAltitudeConstants.VisionConstants.LIMELIGHT_POLL_RATE_HZ;
        if (!Double.isFinite(requestedRate) || requestedRate < 1.0 || requestedRate > 250.0) {
            RobotLog.addGlobalWarningMessage(
                    "Limelight poll rate must be finite and within SDK range 1..250 Hz");
            return false;
        }
        try {
            limelight.setPollRateHz((int) Math.round(requestedRate));
            return requestedPipelineIndex >= 0;
        } catch (RuntimeException exception) {
            RobotLog.addGlobalWarningMessage("Limelight configuration failed closed");
            return false;
        }
    }

    private void publishUnavailableSafely() {
        boolean connected = false;
        try {
            connected = limelight != null && limelight.isConnected();
        } catch (RuntimeException ignored) {
            // Diagnostic state remains disconnected when the SDK itself faults.
        }
        latestRaw = LimelightRawSample.unavailable(limelight != null, connected,
                System.nanoTime());
        latest = classifyRaw(latestRaw);
    }

    /** Alias de lifecycle para cleanup de recursos y bloques try-with-resources. */
    @Override
    public void close() {
        stop();
    }
}
