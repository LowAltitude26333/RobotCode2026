package org.firstinspires.ftc.teamcode.safety;

/**
 * Pure state machine for FND-018 (feeder sin pulso máximo/cooldown). Bounds how long a
 * single feed pulse may stay energized regardless of how long the caller keeps requesting
 * it (e.g. a driver holding a hold-to-run button), and enforces a mandatory cooldown
 * before the next pulse is allowed. Mirrors the testable-with-explicit-nowNanos pattern
 * already used by {@code TurretArmingStateMachine}.
 *
 * Readiness (shooter RPM, health, etc.) is NOT this class's concern — that stays owned by
 * the caller, same as the existing FND-005 fix. This machine only bounds duration/cooldown
 * of whatever pulse the caller already decided to request.
 */
public final class FeederPulseStateMachine {
    public enum State {
        IDLE,
        PULSING,
        COOLDOWN,
        FAULT
    }

    private final long maxPulseNanos;
    private final long cooldownNanos;

    private State state = State.IDLE;
    private long stateEnteredNanos;

    public FeederPulseStateMachine(long maxPulseMillis, long cooldownMillis) {
        if (maxPulseMillis <= 0 || cooldownMillis < 0) {
            throw new IllegalArgumentException(
                    "maxPulseMillis must be positive and cooldownMillis must be non-negative");
        }
        this.maxPulseNanos = maxPulseMillis * 1_000_000L;
        this.cooldownNanos = cooldownMillis * 1_000_000L;
    }

    /**
     * Call every cycle the caller wants the feeder energized. Returns whether power
     * should actually be applied this cycle: true only while PULSING and under the max
     * pulse duration. A request received during COOLDOWN or after the pulse budget is
     * exhausted is silently denied (returns false) until {@link #release} is called.
     */
    public boolean requestPulse(long nowNanos) {
        update(nowNanos);
        switch (state) {
            case IDLE:
                state = State.PULSING;
                stateEnteredNanos = nowNanos;
                return true;
            case PULSING:
                if (elapsedNanos(nowNanos) >= maxPulseNanos) {
                    enterCooldown(nowNanos);
                    return false;
                }
                return true;
            case COOLDOWN:
                return false;
            case FAULT:
                return false;
            default:
                state = State.FAULT;
                return false;
        }
    }

    /** Caller released the request (button up, or command ending). Never re-arms mid-cooldown. */
    public void release(long nowNanos) {
        if (state == State.PULSING) {
            enterCooldown(nowNanos);
        }
    }

    /**
     * Advances PULSING -> COOLDOWN at the hard pulse limit and COOLDOWN -> IDLE once
     * cooldown elapses. Call once per scheduler cycle so the hard cutoff does not depend
     * on the caller issuing another pulse request.
     */
    public void update(long nowNanos) {
        if (state == State.PULSING && elapsedNanos(nowNanos) >= maxPulseNanos) {
            enterCooldown(nowNanos);
            return;
        }
        if (state == State.COOLDOWN && elapsedNanos(nowNanos) >= cooldownNanos) {
            state = State.IDLE;
            stateEnteredNanos = nowNanos;
        }
    }

    public State getState() {
        return state;
    }

    private void enterCooldown(long nowNanos) {
        state = State.COOLDOWN;
        stateEnteredNanos = nowNanos;
    }

    private long elapsedNanos(long nowNanos) {
        return Math.max(0, nowNanos - stateEnteredNanos);
    }
}
