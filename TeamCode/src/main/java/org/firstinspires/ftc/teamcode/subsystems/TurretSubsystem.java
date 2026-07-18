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

    private static final double COMMISSIONING_MAX_POWER = 0.05;
    private static final long COMMISSIONING_INPUT_WATCHDOG_NANOS = 100_000_000L;

    private final DcMotor turretMotor;
    private boolean centeredAndArmed;
    private ZeroState zeroState = ZeroState.INVALID_INIT;
    private boolean commissioningMoveActive;
    private boolean commissioningBlockedUntilRelease;
    private double commissioningRequestedPower;
    private long commissioningMoveDeadlineNanos;
    private long commissioningLastInputNanos;
    private double lastAppliedPower;
    private String lastCommissioningResult = "NOT_RUN";

    // --- CONFIGURACIÓN DE LÍMITES (CALIBRADOS) ---
    // IMPORTANTE: Ajusta estos números según lo que anotaste en tus pruebas
    public static final int LIMIT_LEFT = -200;
    public static final int LIMIT_RIGHT = 200;

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

    /** Invalidates the manual zero and immediately commands zero output. */
    public void simulateZeroLossForCommissioning() {
        invalidateZero(ZeroState.INVALID_SIMULATED_RESET, "SIMULATED_ZERO_LOSS");
    }

    public void disarm() {
        invalidateZero(ZeroState.INVALID_STOP, "DISARMED_BY_STOP");
    }

    private void invalidateZero(ZeroState newState, String result) {
        centeredAndArmed = false;
        zeroState = newState;
        commissioningMoveActive = false;
        commissioningBlockedUntilRelease = true;
        lastCommissioningResult = result;
        stop();
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
        if (!commissioningMoveActive) {
            commissioningMoveActive = true;
            commissioningRequestedPower = safePower;
            commissioningMoveDeadlineNanos = nowNanos + maxDurationMillis * 1_000_000L;
            lastCommissioningResult = "JOG_ACTIVE";
        } else if (Math.signum(safePower) != Math.signum(commissioningRequestedPower)) {
            finishCommissioningMove("STOPPED_DIRECTION_CHANGE", true);
            return;
        }

        commissioningLastInputNanos = nowNanos;
        if (isOutwardAtSoftLimit(turretMotor.getCurrentPosition(), safePower)) {
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
        }
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

        // BLOQUEO DE SEGURIDAD
        // Si intenta ir a la izquierda (potencia negativa) y ya pasó el límite izquierdo
        if (currentPos <= LIMIT_LEFT && power < 0) {
            applyPower(0);
        }
        // Si intenta ir a la derecha (potencia positiva) y ya pasó el límite derecho
        else if (currentPos >= LIMIT_RIGHT && power > 0) {
            applyPower(0);
        }
        // De lo contrario, movimiento libre
        else {
            applyPower(power);
        }
    }

    public double getPosition() {
        return turretMotor.getCurrentPosition();
    }

    public void stop() {
        commissioningMoveActive = false;
        applyPower(0);
        if (turretMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void periodic() {
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

    private boolean isOutwardAtSoftLimit(int currentTicks, double requestedPower) {
        return (currentTicks <= LIMIT_LEFT && requestedPower < 0)
                || (currentTicks >= LIMIT_RIGHT && requestedPower > 0);
    }

    private void finishCommissioningMove(String result, boolean blockUntilRelease) {
        commissioningMoveActive = false;
        commissioningBlockedUntilRelease = blockUntilRelease;
        applyPower(0);
        lastCommissioningResult = result;
    }

    private void applyPower(double power) {
        lastAppliedPower = power;
        turretMotor.setPower(power);
    }
}
