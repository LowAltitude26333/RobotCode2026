package org.firstinspires.ftc.teamcode.commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootBurstCommand extends CommandBase {
    private enum State {
        WAITING_FOR_READY,
        KICKING,
        RETRACT_DELAY,
        FINISHED
    }

    private final ShooterSubsystem shooter;
    private final ShooterHoodSubsystem hood;
    private final KickerSubsystem kicker;
    private final int requestedShots;
    private final LowAltitudeConstants.TargetRPM targetRpm;
    private final LowAltitudeConstants.HoodPosition targetAngle;
    private final Runnable failureCallback;

    private State state;
    private long stateDeadlineNanos;
    private int completedShots;
    private int readyAttempt;
    private boolean failed;

    public ShootBurstCommand(ShooterSubsystem shooter,
                             ShooterHoodSubsystem hood,
                             KickerSubsystem kicker,
                             int shots,
                             LowAltitudeConstants.TargetRPM targetRpm,
                             LowAltitudeConstants.HoodPosition targetAngle,
                             Runnable failureCallback) {
        this.shooter = shooter;
        this.hood = hood;
        this.kicker = kicker;
        this.requestedShots = Math.max(0, shots);
        this.targetRpm = targetRpm;
        this.targetAngle = targetAngle;
        this.failureCallback = failureCallback;
        addRequirements(shooter, hood, kicker);
    }

    public ShootBurstCommand(ShooterSubsystem shooter,
                             ShooterHoodSubsystem hood,
                             KickerSubsystem kicker,
                             int shots,
                             LowAltitudeConstants.TargetRPM targetRpm,
                             LowAltitudeConstants.HoodPosition targetAngle) {
        this(shooter, hood, kicker, shots, targetRpm, targetAngle, () -> { });
    }

    public ShootBurstCommand(ShooterSubsystem shooter,
                             ShooterHoodSubsystem hood,
                             KickerSubsystem kicker,
                             int shots) {
        this(shooter, hood, kicker, shots,
                LowAltitudeConstants.TargetRPM.SHORT_SHOT_RPM,
                LowAltitudeConstants.HoodPosition.SHORT_SHOT);
    }

    @Override
    public void initialize() {
        completedShots = 0;
        readyAttempt = 1;
        failed = false;
        hood.setAngle(targetAngle.angle);
        shooter.setTargetEnum(targetRpm);

        if (requestedShots == 0) {
            state = State.FINISHED;
        } else {
            beginReadyWindow();
        }
    }

    private void beginReadyWindow() {
        state = State.WAITING_FOR_READY;
        stateDeadlineNanos = System.nanoTime()
                + Math.max(1, LowAltitudeConstants.SHOOTER_READY_TIMEOUT_MS) * 1_000_000L;
    }

    @Override
    public void execute() {
        long now = System.nanoTime();

        switch (state) {
            case WAITING_FOR_READY:
                if (shooter.isReady()) {
                    kicker.kick();
                    state = State.KICKING;
                    stateDeadlineNanos = now
                            + Math.max(0, LowAltitudeConstants.KICKER_EXTEND_TIME_MS) * 1_000_000L;
                } else if (now >= stateDeadlineNanos) {
                    if (readyAttempt < Math.max(1,
                            LowAltitudeConstants.SHOOTER_READY_MAX_ATTEMPTS)) {
                        readyAttempt++;
                        shooter.setTargetEnum(targetRpm);
                        beginReadyWindow();
                    } else {
                        failBurst();
                    }
                }
                break;

            case KICKING:
                if (now >= stateDeadlineNanos) {
                    kicker.stop();
                    state = State.RETRACT_DELAY;
                    stateDeadlineNanos = now
                            + Math.max(0,
                            LowAltitudeConstants.KICKER_RETRACT_DELAY_MS) * 1_000_000L;
                }
                break;

            case RETRACT_DELAY:
                if (now >= stateDeadlineNanos) {
                    completedShots++;
                    if (completedShots >= requestedShots) {
                        state = State.FINISHED;
                    } else {
                        readyAttempt = 1;
                        beginReadyWindow();
                    }
                }
                break;

            case FINISHED:
                break;
        }
    }

    private void failBurst() {
        failed = true;
        kicker.stop();
        shooter.stop();
        state = State.FINISHED;
        failureCallback.run();
    }

    public boolean hasFailed() {
        return failed;
    }

    public int getReadyAttempt() {
        return readyAttempt;
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
