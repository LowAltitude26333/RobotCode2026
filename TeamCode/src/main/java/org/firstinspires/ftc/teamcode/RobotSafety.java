package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

/** Collects idempotent actuator shutdown hooks for the active OpMode. */
public final class RobotSafety {
    private static final List<Runnable> shutdownHooks = new ArrayList<>();

    private RobotSafety() {
    }

    public static synchronized void beginOpMode() {
        shutdownHooks.clear();
    }

    public static synchronized void registerShutdown(Runnable shutdownHook) {
        shutdownHooks.add(shutdownHook);
    }

    public static void stopAll() {
        List<Runnable> hooks;
        synchronized (RobotSafety.class) {
            hooks = new ArrayList<>(shutdownHooks);
        }

        for (int i = hooks.size() - 1; i >= 0; i--) {
            try {
                hooks.get(i).run();
            } catch (RuntimeException ignored) {
                // Continue stopping the remaining mechanisms if one device fails.
            }
        }
    }
}
