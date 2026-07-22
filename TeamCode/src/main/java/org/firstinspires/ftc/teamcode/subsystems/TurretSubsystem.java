package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.RobotSafety;

public class TurretSubsystem extends SubsystemBase {
    public enum ZeroState {
        INVALID_INIT,
        VALID_MANUAL_CONFIRMATION,
        INVALID_SIMULATED_RESET,
        INVALID_STOP
    }

    private static final double COMMISSIONING_MAX_POWER = 0.50;
    private static final double LIMIT_APPROACH_POWER = 0.05;
    private static final int LIMIT_SLOW_ZONE_TICKS = 100;
    private static final long COMMISSIONING_INPUT_WATCHDOG_NANOS = 100_000_000L;
    private static final int PRESET_TOLERANCE_TICKS = 5;

    /**
     * Medición inmutable de un jog de commissioning: cuánto duró el hold real y
     * cuántos ticks se movió el encoder entre el inicio y el corte. Existe para
     * explicar la asimetría +12/-62 observada en T4 (FND-027) separando duración
     * humana del D-pad de respuesta mecánica. Solo instrumentación: no cambia
     * potencia, timeout, watchdog ni límites.
     */
    public static final class JogReport {
        public final double requestedPower;
        public final double holdDurationMs;
        public final int startTicks;
        public final int endTicks;
        public final String result;

        JogReport(double requestedPower, double holdDurationMs,
                  int startTicks, int endTicks, String result) {
            this.requestedPower = requestedPower;
            this.holdDurationMs = holdDurationMs;
            this.startTicks = startTicks;
            this.endTicks = endTicks;
            this.result = result;
        }

        public int deltaTicks() {
            return endTicks - startTicks;
        }
    }

    private final DcMotor turretMotor;
    private boolean centeredAndArmed;
    private ZeroState zeroState = ZeroState.INVALID_INIT;
    private boolean commissioningMoveActive;
    private boolean commissioningBlockedUntilRelease;
    private double commissioningRequestedPower;
    private long commissioningMoveDeadlineNanos;
    private long commissioningLastInputNanos;
    private long commissioningMoveStartNanos;
    private int commissioningMoveStartTicks;
    private double lastAppliedPower;
    private String lastCommissioningResult = "NOT_RUN";
    private JogReport lastJogReport;
    private boolean presetMoveActive;
    private int presetTargetTicks;
    private long presetDeadlineNanos;

    // El límite negativo conserva 10 ticks desde el extremo -993. El lead confirmó
    // +1070 como límite positivo final con su tolerancia ya aplicada.
    public static final int LIMIT_LEFT = -983;
    public static final int LIMIT_RIGHT = 1070;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, RobotMap.TURRET_MOTOR);

        // Aplicar inversión desde el RobotMap para que el encoder y el motor coincidan
        if (RobotMap.TURRET_MOTOR_IS_INVERTED) {
            turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // BRAKE ayuda a que la torreta no se pase del límite por inercia
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setPower(0);
        centeredAndArmed = false;
        RobotSafety.registerShutdown(this::disarm);
    }

    public void confirmCenteredAndResetEncoder() {
        stop();
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        centeredAndArmed = true;
        commissioningBlockedUntilRelease = false;
        zeroState = ZeroState.VALID_MANUAL_CONFIRMATION;
        lastCommissioningResult = "ZERO_CONFIRMED";
    }

    public boolean isArmed() {
        return centeredAndArmed;
    }

    public ZeroState getZeroState() {
        return zeroState;
    }

    public boolean isCommissioningMoveActive() {
        return commissioningMoveActive;
    }

    public double getLastAppliedPower() {
        return lastAppliedPower;
    }

    public String getLastCommissioningResult() {
        return lastCommissioningResult;
    }

    public boolean isPresetMoveActive() {
        return presetMoveActive;
    }

    public int getPresetTargetTicks() {
        return presetTargetTicks;
    }

    /** Última medición de jog completada; null hasta terminar el primer jog. */
    public JogReport getLastJogReport() {
        return lastJogReport;
    }

    /** Invalidates the manual zero and immediately commands zero output. */
    public void simulateZeroLossForCommissioning() {
        invalidateZero(ZeroState.INVALID_SIMULATED_RESET, "SIMULATED_ZERO_LOSS");
    }

    public void disarm() {
        invalidateZero(ZeroState.INVALID_STOP, "DISARMED_BY_STOP");
    }

    private void invalidateZero(ZeroState newState, String result) {
        boolean jogWasActive = commissioningMoveActive;
        centeredAndArmed = false;
        zeroState = newState;
        commissioningMoveActive = false;
        commissioningBlockedUntilRelease = true;
        lastCommissioningResult = result;
        stop();
        if (jogWasActive) {
            // Un Stop/E-stop que corta un jog activo también cierra su medición.
            recordJogReport(result);
        }
    }

    /**
     * Hold-to-run commissioning jog. The caller refreshes this request each control cycle.
     * This API keeps the test inside a confirmed zero, low power, soft limits, and timeout.
     */
    public void requestCommissioningJog(double requestedPower, long maxDurationMillis) {
        long nowNanos = System.nanoTime();

        if (!centeredAndArmed || zeroState != ZeroState.VALID_MANUAL_CONFIRMATION) {
            finishCommissioningMove("REJECTED_ZERO_INVALID", true);
            return;
        }
        if (!Double.isFinite(requestedPower)
                || requestedPower == 0
                || maxDurationMillis <= 0) {
            finishCommissioningMove("REJECTED_INVALID_REQUEST", true);
            return;
        }
        if (commissioningBlockedUntilRelease) {
            applyPower(0);
            return;
        }

        double safePower = Math.max(-COMMISSIONING_MAX_POWER,
                Math.min(COMMISSIONING_MAX_POWER, requestedPower));
        int currentTicks = turretMotor.getCurrentPosition();
        safePower = applyLimitApproachPower(currentTicks, safePower);
        if (!commissioningMoveActive) {
            commissioningMoveActive = true;
            commissioningRequestedPower = safePower;
            commissioningMoveDeadlineNanos = nowNanos + maxDurationMillis * 1_000_000L;
            // Instrumentación FND-027: marca de inicio del hold para medir
            // duración real y delta de ticks. No altera el control.
            commissioningMoveStartNanos = nowNanos;
            commissioningMoveStartTicks = currentTicks;
            lastCommissioningResult = "JOG_ACTIVE";
        } else if (Math.signum(safePower) != Math.signum(commissioningRequestedPower)) {
            finishCommissioningMove("STOPPED_DIRECTION_CHANGE", true);
            return;
        }

        commissioningLastInputNanos = nowNanos;
        if (isOutwardAtSoftLimit(currentTicks, safePower)) {
            finishCommissioningMove("STOPPED_SOFT_LIMIT", true);
        } else if (nowNanos >= commissioningMoveDeadlineNanos) {
            finishCommissioningMove("STOPPED_TIMEOUT", true);
        } else {
            applyPower(safePower);
        }
    }

    /** Stops immediately and rearms commissioning only after the D-pad is released. */
    public void releaseCommissioningJog() {
        boolean wasActive = commissioningMoveActive;
        commissioningMoveActive = false;
        commissioningBlockedUntilRelease = false;
        applyPower(0);
        if (wasActive) {
            lastCommissioningResult = "STOPPED_RELEASE";
            recordJogReport("STOPPED_RELEASE");
        }
    }

    /** Moves to a bounded encoder preset after the operator confirms the physical zero. */
    public boolean requestPresetPosition(int targetTicks, long maxDurationMillis) {
        if (!centeredAndArmed || zeroState != ZeroState.VALID_MANUAL_CONFIRMATION) {
            lastCommissioningResult = "REJECTED_ZERO_INVALID";
            stop();
            return false;
        }
        if (targetTicks < LIMIT_LEFT || targetTicks > LIMIT_RIGHT || maxDurationMillis <= 0) {
            lastCommissioningResult = "REJECTED_PRESET_INVALID";
            stop();
            return false;
        }

        commissioningMoveActive = false;
        commissioningBlockedUntilRelease = false;
        presetTargetTicks = targetTicks;
        presetDeadlineNanos = System.nanoTime() + maxDurationMillis * 1_000_000L;
        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        presetMoveActive = true;
        lastCommissioningResult = "PRESET_ACTIVE";
        applyPower(presetPowerForPosition(turretMotor.getCurrentPosition()));
        return true;
    }

    /**
     * Establece la potencia del motor verificando los límites del encoder.
     * @param power Potencia solicitada (-1.0 a 1.0)
     */
    public void setPower(double power) {
        if (!centeredAndArmed) {
            applyPower(0);
            return;
        }

        int currentPos = (int) getPosition();
        double safePower = Math.max(-COMMISSIONING_MAX_POWER,
                Math.min(COMMISSIONING_MAX_POWER, power));
        safePower = applyLimitApproachPower(currentPos, safePower);

        // BLOQUEO DE SEGURIDAD
        // Si intenta ir a la izquierda (potencia negativa) y ya pasó el límite izquierdo
        if (currentPos <= LIMIT_LEFT && safePower < 0) {
            applyPower(0);
        }
        // Si intenta ir a la derecha (potencia positiva) y ya pasó el límite derecho
        else if (currentPos >= LIMIT_RIGHT && safePower > 0) {
            applyPower(0);
        }
        // De lo contrario, movimiento libre
        else {
            applyPower(safePower);
        }
    }

    public double getPosition() {
        return turretMotor.getCurrentPosition();
    }

    public void stop() {
        commissioningMoveActive = false;
        presetMoveActive = false;
        applyPower(0);
        if (turretMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void periodic() {
        if (presetMoveActive) {
            int currentTicks = turretMotor.getCurrentPosition();
            if (!centeredAndArmed || zeroState != ZeroState.VALID_MANUAL_CONFIRMATION) {
                finishPresetMove("STOPPED_ZERO_INVALID");
            } else if (currentTicks < LIMIT_LEFT || currentTicks > LIMIT_RIGHT) {
                finishPresetMove("STOPPED_SOFT_LIMIT");
            } else if (Math.abs(presetTargetTicks - currentTicks) <= PRESET_TOLERANCE_TICKS
                    || !turretMotor.isBusy()) {
                finishPresetMove("PRESET_REACHED");
            } else if (System.nanoTime() >= presetDeadlineNanos) {
                finishPresetMove("STOPPED_PRESET_TIMEOUT");
            } else {
                applyPower(presetPowerForPosition(currentTicks));
            }
            return;
        }
        if (!commissioningMoveActive) {
            return;
        }

        long nowNanos = System.nanoTime();
        int currentTicks = turretMotor.getCurrentPosition();
        if (!centeredAndArmed) {
            finishCommissioningMove("STOPPED_ZERO_INVALID", true);
        } else if (isOutwardAtSoftLimit(currentTicks, commissioningRequestedPower)) {
            finishCommissioningMove("STOPPED_SOFT_LIMIT", true);
        } else if (nowNanos >= commissioningMoveDeadlineNanos) {
            finishCommissioningMove("STOPPED_TIMEOUT", true);
        } else if (nowNanos - commissioningLastInputNanos
                > COMMISSIONING_INPUT_WATCHDOG_NANOS) {
            finishCommissioningMove("STOPPED_INPUT_WATCHDOG", true);
        }
    }

    private void finishPresetMove(String result) {
        stop();
        lastCommissioningResult = result;
    }

    private boolean isOutwardAtSoftLimit(int currentTicks, double requestedPower) {
        return (currentTicks <= LIMIT_LEFT && requestedPower < 0)
                || (currentTicks >= LIMIT_RIGHT && requestedPower > 0);
    }

    private double applyLimitApproachPower(int currentTicks, double requestedPower) {
        if (requestedPower < 0 && currentTicks <= LIMIT_LEFT + LIMIT_SLOW_ZONE_TICKS) {
            return -Math.min(Math.abs(requestedPower), LIMIT_APPROACH_POWER);
        }
        if (requestedPower > 0 && currentTicks >= LIMIT_RIGHT - LIMIT_SLOW_ZONE_TICKS) {
            return Math.min(Math.abs(requestedPower), LIMIT_APPROACH_POWER);
        }
        return requestedPower;
    }

    private double presetPowerForPosition(int currentTicks) {
        double direction = Math.signum(presetTargetTicks - currentTicks);
        if (direction == 0) {
            return 0;
        }
        if (Math.abs(presetTargetTicks - currentTicks) <= LIMIT_SLOW_ZONE_TICKS) {
            return direction * LIMIT_APPROACH_POWER;
        }
        return direction * COMMISSIONING_MAX_POWER;
    }

    private void finishCommissioningMove(String result, boolean blockUntilRelease) {
        boolean wasActive = commissioningMoveActive;
        commissioningMoveActive = false;
        commissioningBlockedUntilRelease = blockUntilRelease;
        applyPower(0);
        lastCommissioningResult = result;
        if (wasActive) {
            recordJogReport(result);
        }
    }

    /**
     * Cierra la medición del jog que estaba activo. Se llama DESPUÉS de ordenar
     * potencia cero, así que la lectura de ticks es la del momento del corte;
     * la deriva posterior por inercia se observa en la telemetría en vivo.
     */
    private void recordJogReport(String result) {
        long nowNanos = System.nanoTime();
        lastJogReport = new JogReport(
                commissioningRequestedPower,
                (nowNanos - commissioningMoveStartNanos) / 1_000_000.0,
                commissioningMoveStartTicks,
                turretMotor.getCurrentPosition(),
                result);
    }

    private void applyPower(double power) {
        lastAppliedPower = power;
        turretMotor.setPower(power);
    }
}
