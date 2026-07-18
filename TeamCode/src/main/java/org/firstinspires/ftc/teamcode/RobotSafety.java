package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

/** Collects idempotent actuator shutdown hooks for the active OpMode. */
public final class RobotSafety {
    public static final long STOP_DEADLINE_NANOS = 50_000_000L;

    public static final class StopTimingReport {
        private final String event;
        private final long elapsedNanos;
        private final int stopActionCount;
        private final int failureCount;
        private final boolean sameControlCycle;

        private StopTimingReport(String event, long elapsedNanos, int stopActionCount,
                                 int failureCount, boolean sameControlCycle) {
            this.event = event;
            this.elapsedNanos = elapsedNanos;
            this.stopActionCount = stopActionCount;
            this.failureCount = failureCount;
            this.sameControlCycle = sameControlCycle;
        }

        public String getEvent() {
            return event;
        }

        public double getElapsedMilliseconds() {
            return elapsedNanos / 1_000_000.0;
        }

        public int getStopActionCount() {
            return stopActionCount;
        }

        public int getFailureCount() {
            return failureCount;
        }

        public boolean isSameControlCycle() {
            return sameControlCycle;
        }

        public boolean passed() {
            return sameControlCycle
                    && failureCount == 0
                    && elapsedNanos <= STOP_DEADLINE_NANOS;
        }
    }

    private static final List<Runnable> shutdownHooks = new ArrayList<>();
    private static volatile StopTimingReport lastStopTimingReport;

    private RobotSafety() {
    }

    public static synchronized void beginOpMode() {
        shutdownHooks.clear();
    }

    public static synchronized void registerShutdown(Runnable shutdownHook) {
        shutdownHooks.add(shutdownHook);
    }

    public static void stopAll() {
        runShutdownHooks(null, System.nanoTime(), false);
    }

    public static StopTimingReport stopAllTimed(String event, long requestNanos) {
        return runShutdownHooks(event, requestNanos, true);
    }

    public static StopTimingReport timeZeroCommand(String event, Runnable zeroCommand) {
        long requestNanos = System.nanoTime();
        int failureCount = 0;
        RuntimeException failure = null;

        try {
            zeroCommand.run();
        } catch (RuntimeException exception) {
            failureCount = 1;
            failure = exception;
        }

        StopTimingReport report = publishReport(
                event, requestNanos, 1, failureCount, true);
        if (failure != null) {
            throw failure;
        }
        return report;
    }

    public static StopTimingReport getLastStopTimingReport() {
        return lastStopTimingReport;
    }

    private static StopTimingReport runShutdownHooks(String event, long requestNanos,
                                                     boolean publishTiming) {
        List<Runnable> hooks;
        synchronized (RobotSafety.class) {
            hooks = new ArrayList<>(shutdownHooks);
        }

        int failureCount = 0;
        for (int i = hooks.size() - 1; i >= 0; i--) {
            try {
                hooks.get(i).run();
            } catch (RuntimeException ignored) {
                failureCount++;
                // Continue stopping the remaining mechanisms if one device fails.
            }
        }

        if (!publishTiming) {
            return lastStopTimingReport;
        }
        return publishReport(event, requestNanos, hooks.size(), failureCount, true);
    }

    private static StopTimingReport publishReport(String event, long requestNanos,
                                                  int stopActionCount, int failureCount,
                                                  boolean sameControlCycle) {
        long elapsedNanos = Math.max(0, System.nanoTime() - requestNanos);
        StopTimingReport report = new StopTimingReport(
                event, elapsedNanos, stopActionCount, failureCount, sameControlCycle);
        lastStopTimingReport = report;
        return report;
    }
}
