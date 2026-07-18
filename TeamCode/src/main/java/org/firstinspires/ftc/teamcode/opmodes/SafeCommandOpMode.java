package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.RobotSafety;

import java.util.ArrayList;
import java.util.List;

/** CommandOpMode lifecycle that always interrupts commands and stops actuators. */
public abstract class SafeCommandOpMode extends CommandOpMode {
    private final List<Runnable> resourceCleanupHooks = new ArrayList<>();
    private final List<Command> scheduledTopLevelCommands = new ArrayList<>();
    private boolean emergencyStopLatched;

    /** Called repeatedly after initialize() and before Start, without running the scheduler. */
    protected void duringInitLoop() {
    }

    protected final void addResourceCleanup(Runnable cleanupHook) {
        resourceCleanupHooks.add(cleanupHook);
    }

    @Override
    public final void schedule(Command... commands) {
        for (Command command : commands) {
            if (!scheduledTopLevelCommands.contains(command)) {
                scheduledTopLevelCommands.add(command);
            }
        }
        super.schedule(commands);
    }

    protected final void cancelScheduledCommands() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        for (Command command : new ArrayList<>(scheduledTopLevelCommands)) {
            scheduler.cancel(command);
        }
    }

    protected final void latchEmergencyStop() {
        long requestNanos = System.nanoTime();
        emergencyStopLatched = true;
        cancelScheduledCommands();
        CommandScheduler.getInstance().disable();
        RobotSafety.stopAllTimed("E_STOP", requestNanos);
    }

    private boolean handleEmergencyStopRequest() {
        if (!emergencyStopLatched && gamepad1.back) {
            latchEmergencyStop();
            requestOpModeStop();
        }
        return emergencyStopLatched;
    }

    protected final void addSafetyTimingTelemetry() {
        RobotSafety.StopTimingReport report = RobotSafety.getLastStopTimingReport();
        if (report == null) {
            telemetry.addData("Safety timing", "SIN MEDICIÓN");
            return;
        }

        telemetry.addData("Safety/Evento", report.getEvent());
        telemetry.addData("Safety/Tiempo cero", "%.3f ms", report.getElapsedMilliseconds());
        telemetry.addData("Safety/Mismo ciclo", report.isSameControlCycle());
        telemetry.addData("Safety/Acciones stop", report.getStopActionCount());
        telemetry.addData("Safety/Fallos stop", report.getFailureCount());
        telemetry.addData("Safety/Gate 50 ms", report.passed() ? "PASS" : "FAIL");
    }

    @Override
    public void run() {
        super.run();
        addSafetyTimingTelemetry();
    }

    @Override
    public final void runOpMode() throws InterruptedException {
        RobotSafety.beginOpMode();
        CommandScheduler.getInstance().enable();
        emergencyStopLatched = false;

        try {
            initialize();

            while (!isStarted() && !isStopRequested()) {
                if (handleEmergencyStopRequest()) {
                    break;
                }
                duringInitLoop();
                addSafetyTimingTelemetry();
                telemetry.update();
                idle();
            }

            while (!isStopRequested() && opModeIsActive()) {
                if (handleEmergencyStopRequest()) {
                    break;
                }
                run();
            }
        } finally {
            long shutdownRequestNanos = System.nanoTime();
            cancelScheduledCommands();
            CommandScheduler.getInstance().disable();
            if (emergencyStopLatched) {
                // Preserve the E_STOP report while still issuing an idempotent final zero.
                RobotSafety.stopAll();
            } else {
                RobotSafety.stopAllTimed("OPMODE_STOP", shutdownRequestNanos);
            }

            for (int i = resourceCleanupHooks.size() - 1; i >= 0; i--) {
                try {
                    resourceCleanupHooks.get(i).run();
                } catch (RuntimeException ignored) {
                    // Attempt every cleanup even when one resource reports an error.
                }
            }

            reset();
        }
    }
}
