package org.firstinspires.ftc.teamcode.safety;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class FeederPulseStateMachineTest {

    private static final long MAX_PULSE_MS = 500;
    private static final long COOLDOWN_MS = 200;

    private FeederPulseStateMachine newMachine() {
        return new FeederPulseStateMachine(MAX_PULSE_MS, COOLDOWN_MS);
    }

    @Test
    public void rejectsNonPositiveMaxPulseOrNegativeCooldown() {
        try {
            new FeederPulseStateMachine(0, 100);
            org.junit.Assert.fail("expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            // expected
        }
        try {
            new FeederPulseStateMachine(500, -1);
            org.junit.Assert.fail("expected IllegalArgumentException");
        } catch (IllegalArgumentException expected) {
            // expected
        }
    }

    @Test
    public void firstRequestFromIdleStartsPulsingAndAllowsPower() {
        FeederPulseStateMachine machine = newMachine();
        assertEquals(FeederPulseStateMachine.State.IDLE, machine.getState());

        assertTrue(machine.requestPulse(0L));
        assertEquals(FeederPulseStateMachine.State.PULSING, machine.getState());
    }

    @Test
    public void pulseKeepsAllowingPowerUntilMaxPulseElapsed() {
        FeederPulseStateMachine machine = newMachine();
        long startNanos = 1_000_000_000L;
        assertTrue(machine.requestPulse(startNanos));

        long justUnderMax = startNanos + (MAX_PULSE_MS - 1) * 1_000_000L;
        assertTrue(machine.requestPulse(justUnderMax));
        assertEquals(FeederPulseStateMachine.State.PULSING, machine.getState());
    }

    @Test
    public void pulseCutsItselfAtMaxDurationEvenIfCallerKeepsRequesting() {
        FeederPulseStateMachine machine = newMachine();
        long startNanos = 1_000_000_000L;
        assertTrue(machine.requestPulse(startNanos));

        long atOrPastMax = startNanos + MAX_PULSE_MS * 1_000_000L;
        assertFalse(machine.requestPulse(atOrPastMax));
        assertEquals(FeederPulseStateMachine.State.COOLDOWN, machine.getState());

        // Still held by the caller a moment later: stays denied through cooldown.
        assertFalse(machine.requestPulse(atOrPastMax + 1_000_000L));
        assertEquals(FeederPulseStateMachine.State.COOLDOWN, machine.getState());
    }

    @Test
    public void periodicUpdateCutsPulseWithoutAnotherRequest() {
        FeederPulseStateMachine machine = newMachine();
        long startNanos = 1_000_000_000L;
        assertTrue(machine.requestPulse(startNanos));

        machine.update(startNanos + MAX_PULSE_MS * 1_000_000L);

        assertEquals(FeederPulseStateMachine.State.COOLDOWN, machine.getState());
    }

    @Test
    public void zeroCooldownStillObservesOneCutoffTransition() {
        FeederPulseStateMachine machine = new FeederPulseStateMachine(MAX_PULSE_MS, 0);
        assertTrue(machine.requestPulse(0L));

        machine.update(MAX_PULSE_MS * 1_000_000L);
        assertEquals(FeederPulseStateMachine.State.COOLDOWN, machine.getState());

        machine.update(MAX_PULSE_MS * 1_000_000L + 1L);
        assertEquals(FeederPulseStateMachine.State.IDLE, machine.getState());
    }

    @Test
    public void requestDuringCooldownIsDeniedUntilCooldownElapses() {
        FeederPulseStateMachine machine = newMachine();
        long startNanos = 0L;
        assertTrue(machine.requestPulse(startNanos));
        machine.release(startNanos + 50L * 1_000_000L); // release mid-pulse -> COOLDOWN

        long stillCooling = startNanos + 50L * 1_000_000L + (COOLDOWN_MS - 1) * 1_000_000L;
        assertFalse(machine.requestPulse(stillCooling));
        assertEquals(FeederPulseStateMachine.State.COOLDOWN, machine.getState());

        long cooldownDone = startNanos + 50L * 1_000_000L + (COOLDOWN_MS + 1) * 1_000_000L;
        assertTrue(machine.requestPulse(cooldownDone));
        assertEquals(FeederPulseStateMachine.State.PULSING, machine.getState());
    }

    @Test
    public void releaseWhileIdleOrCoolingDownIsANoOp() {
        FeederPulseStateMachine machine = newMachine();
        machine.release(0L); // never pulsed; must not throw or change state
        assertEquals(FeederPulseStateMachine.State.IDLE, machine.getState());
    }

    @Test
    public void updateAdvancesCooldownToIdleWithoutARequest() {
        FeederPulseStateMachine machine = newMachine();
        assertTrue(machine.requestPulse(0L));
        machine.release(10L * 1_000_000L);
        assertEquals(FeederPulseStateMachine.State.COOLDOWN, machine.getState());

        machine.update(10L * 1_000_000L + (COOLDOWN_MS - 1) * 1_000_000L);
        assertEquals(FeederPulseStateMachine.State.COOLDOWN, machine.getState());

        machine.update(10L * 1_000_000L + (COOLDOWN_MS + 1) * 1_000_000L);
        assertEquals(FeederPulseStateMachine.State.IDLE, machine.getState());
    }
}
