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
        cancelScheduledCommands();
        CommandScheduler.getInstance().disable();
        RobotSafety.stopAll();
    }

    @Override
    public final void runOpMode() throws InterruptedException {
        RobotSafety.beginOpMode();
        CommandScheduler.getInstance().enable();

        try {
            initialize();
            waitForStart();

            while (!isStopRequested() && opModeIsActive()) {
                run();
            }
        } finally {
            cancelScheduledCommands();
            CommandScheduler.getInstance().disable();
            RobotSafety.stopAll();

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
