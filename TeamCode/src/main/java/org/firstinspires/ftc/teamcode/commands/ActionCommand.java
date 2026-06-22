package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class ActionCommand extends CommandBase {
    private final Action action;
    private final Runnable cleanup;
    private boolean finished = false;

    /** Road Runner drive action with deterministic motor cleanup. */
    public ActionCommand(Action action, DriveSubsystem drive) {
        this.action = action;
        this.cleanup = drive::stop;
        addRequirements(drive);
    }

    public ActionCommand(Action action, Runnable cleanup, Subsystem... requirements) {
        this.action = action;
        this.cleanup = java.util.Objects.requireNonNull(cleanup);
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        finished = false;
        // Preview inicial (opcional, ayuda en dashboard)
        TelemetryPacket packet = new TelemetryPacket();
        action.preview(packet.fieldOverlay());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        action.preview(packet.fieldOverlay());

        // RoadRunner: run() devuelve true si sigue corriendo, false si terminó.
        boolean running = action.run(packet);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        finished = !running;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        cleanup.run();
    }
}
