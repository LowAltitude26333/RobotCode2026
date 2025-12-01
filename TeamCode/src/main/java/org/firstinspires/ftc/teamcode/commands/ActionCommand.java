package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import java.util.Set;

public class ActionCommand extends CommandBase {
    private final Action action;
    private boolean finished = false;

    public ActionCommand(Action action, Set<Subsystem> requirements) {
        this.action = action;
        this.m_requirements.addAll(requirements);
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        action.preview(packet.fieldOverlay());
        boolean running = action.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        finished = !running;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}