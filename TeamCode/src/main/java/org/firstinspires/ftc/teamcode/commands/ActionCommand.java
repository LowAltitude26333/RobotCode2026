package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

public class ActionCommand extends CommandBase {
    private final Action action;
    private boolean finished = false;

    // Constructor para acciones genéricas (sin requisitos de subsistema)
    public ActionCommand(Action action) {
        this.action = action;
    }

    // Constructor "PowerHouse": Permite pasar los subsistemas que esta acción usa.
    // EJEMPLO: Para trayectorias, pasas el DriveSubsystem para bloquearlo.
    public ActionCommand(Action action, Subsystem... requirements) {
        this.action = action;
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
}