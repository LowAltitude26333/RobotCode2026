package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;

import org.firstinspires.ftc.teamcode.RobotSafety;

/** Adds a one-way transition from the main autonomous routine to a safe park. */
public abstract class SafeAutonomousOpMode extends SafeCommandOpMode {
    private Command safeParkCommand;
    private boolean safeParkRequested;
    private boolean safeParkScheduled;

    protected final void configureSafePark(Command command) {
        safeParkCommand = command;
    }

    protected final void requestSafePark() {
        safeParkRequested = true;
    }

    @Override
    public final void run() {
        if (safeParkRequested && !safeParkScheduled) {
            cancelScheduledCommands();
            RobotSafety.stopAll();
            safeParkScheduled = true;

            if (safeParkCommand != null) {
                schedule(safeParkCommand);
            }
        }

        super.run();
        afterSchedulerRun();
    }

    protected void afterSchedulerRun() {
    }
}
