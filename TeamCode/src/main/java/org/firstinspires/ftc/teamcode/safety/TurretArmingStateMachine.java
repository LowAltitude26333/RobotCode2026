package org.firstinspires.ftc.teamcode.safety;

/** Pure state machine for confirming a manually centered turret during the init loop. */
public final class TurretArmingStateMachine {
    public enum State {
        WAITING,
        HOLDING,
        ARMED
    }

    private final long holdDurationNanos;
    private State state = State.WAITING;
    private long holdStartedNanos;

    public TurretArmingStateMachine(long holdDurationMillis) {
        if (holdDurationMillis <= 0) {
            throw new IllegalArgumentException("holdDurationMillis must be positive");
        }
        holdDurationNanos = holdDurationMillis * 1_000_000L;
    }

    /** Returns true exactly once, on the transition to ARMED. */
    public boolean update(boolean confirmationChordHeld, long nowNanos) {
        if (state == State.ARMED) {
            return false;
        }
        if (!confirmationChordHeld) {
            state = State.WAITING;
            holdStartedNanos = 0;
            return false;
        }
        if (state == State.WAITING) {
            state = State.HOLDING;
            holdStartedNanos = nowNanos;
            return false;
        }
        if (elapsedNanos(nowNanos) >= holdDurationNanos) {
            state = State.ARMED;
            return true;
        }
        return false;
    }

    public State getState() {
        return state;
    }

    public double getProgress(long nowNanos) {
        if (state == State.ARMED) {
            return 1.0;
        }
        if (state != State.HOLDING) {
            return 0.0;
        }
        return Math.min(1.0, (double) elapsedNanos(nowNanos) / holdDurationNanos);
    }

    private long elapsedNanos(long nowNanos) {
        return Math.max(0, nowNanos - holdStartedNanos);
    }
}
