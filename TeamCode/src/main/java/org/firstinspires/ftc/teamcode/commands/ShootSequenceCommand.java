package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants; // Importar Constantes
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/** Legacy single-shot command retained with a bounded readiness wait. */
public class ShootSequenceCommand extends CommandBase {

    private enum State {
        WAITING_FOR_READY,
        KICKING,
        FINISHED
    }

    private final ShooterSubsystem shooter;
    private final KickerSubsystem kicker;
    private final double targetRpm;
    private State state;
    private long deadlineNanos;

    public ShootSequenceCommand(ShooterSubsystem shooter,
                                ShooterHoodSubsystem hood,
                                KickerSubsystem kicker,
                                double targetRpm,
                                LowAltitudeConstants.HoodPosition angle) {

        this.shooter = shooter;
        this.kicker = kicker;
        this.targetRpm = targetRpm;
        addRequirements(shooter, kicker);
    }

    @Override
    public void initialize() {
        kicker.stop();
        shooter.setTargetRPM(targetRpm);
        state = State.WAITING_FOR_READY;
        deadlineNanos = System.nanoTime()
                + Math.max(1, LowAltitudeConstants.SHOOTER_READY_TIMEOUT_MS) * 1_000_000L;
    }

    @Override
    public void execute() {
        long nowNanos = System.nanoTime();
        if (state == State.WAITING_FOR_READY) {
            if (shooter.isReady()) {
                kicker.kick();
                state = State.KICKING;
                deadlineNanos = nowNanos
                        + Math.max(0, LowAltitudeConstants.KICKER_EXTEND_TIME_MS) * 1_000_000L;
            } else if (nowNanos >= deadlineNanos || shooter.isFaultLatched()) {
                kicker.stop();
                shooter.stop();
                state = State.FINISHED;
            }
        } else if (state == State.KICKING && nowNanos >= deadlineNanos) {
            kicker.stop();
            state = State.FINISHED;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.FINISHED;
    }

    @Override
    public void end(boolean interrupted) {
        kicker.stop();
        if (interrupted) {
            shooter.stop();
        }
    }
}
