package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotSafety;
import org.firstinspires.ftc.teamcode.opmodes.SafeCommandOpMode;
import org.firstinspires.ftc.teamcode.safety.FeederPulseStateMachine;
import org.firstinspires.ftc.teamcode.safety.TurretArmingStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Config // ¡Crucial! Permite editar variables estáticas desde Dashboard
@TeleOp(name = "SYSTEM CHECK and TUNING", group = "Test")
public class SystemCheckOpMode extends SafeCommandOpMode {

    // --- VARIABLES EDITABLES EN DASHBOARD ---
    // Edita esto en vivo en la sección "SystemCheckOpMode" del Dashboard
    // First physical shooter gate: fixed values, deliberately not Dashboard-editable.
    public static final double SHOOTER_COMMISSIONING_TARGET_RPM = 2000.0;
    public static final double SHOOTER_COMMISSIONING_MAX_POWER = 0.90;
    public static final long SHOOTER_COMMISSIONING_PULSE_MS = 8000;
    public static final long SHOOTER_ENCODER_RESPONSE_TIMEOUT_MS = 150;
    public static final double BURST_READY_HOLD_MS = 250.0;
    public static final int BURST_SHOT_COUNT = 3;
    public static final double BURST_MAX_DURATION_MS = 3500.0;
    // Fail closed below the lowest voltage already observed in the accepted one-piece test.
    public static final double BURST_MIN_BATTERY_V = 10.50;
    // MP-01 T4/FND-027: hold-to-run jog with an automatic, repeatable comparison cutoff.
    // Releasing early still commands zero immediately; the subsystem blocks reactivation
    // after the cutoff until the D-pad is released.
    public static final double TURRET_COMMISSIONING_POWER = 0.50;
    public static final long TURRET_COMPARISON_PULSE_MS = 850;

    // Subsistemas
    private ShooterSubsystem shooter;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;
    private TurretSubsystem turret;
    private TurretArmingStateMachine turretArming;

    private GamepadEx driverPad;
    private GamepadEx faultPad;
    private boolean shooterEnabled;
    private boolean shooterPulseActive;
    private boolean shooterPulseConsumed;
    private boolean shooterBlockedUntilRelease;
    private long shooterPulseStartNanos;
    private int shooterPulseStartTicks;
    private int shooterPulseEndTicks;
    private double shooterPulsePeakAbsRpm;
    private double shooterPulseEndIndicatedRpm;
    private double shooterPulseStartVoltage;
    private double shooterPulseEndVoltage;
    private double shooterPulseMinVoltage;
    private double shooterPulseDurationMs;
    private boolean shooterEncoderResponded;
    private long shooterReadyHoldStartNanos;
    private double shooterCurrentReadyHoldMs;
    private double shooterMaxReadyHoldMs;
    private String shooterLastResult = "NOT_RUN";
    private boolean burstActive;
    private boolean burstConsumed;
    private boolean burstRequestHeld;
    private boolean burstShotPulseActive;
    private int burstCompletedShots;
    private long burstStartNanos;
    private long burstShotStartNanos;
    private double burstDurationMs;
    private final double[] burstPreRpm = new double[BURST_SHOT_COUNT];
    private final double[] burstMinRpm = new double[BURST_SHOT_COUNT];
    private final double[] burstShotStartMs = new double[BURST_SHOT_COUNT];
    private String burstResult = "NOT_RUN";

    @Override
    public void initialize() {
        // Combinar telemetría de Driver Station y Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Instanciar Hardware
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        if (shooter.setCommissioningOutputPowerLimit(SHOOTER_COMMISSIONING_MAX_POWER)) {
            shooter.enableCommissioningPhysicalOutput();
        }
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        turretArming = new TurretArmingStateMachine(
                LowAltitudeConstants.TurretConstants.TURRET_ARM_HOLD_MS);

        driverPad = new GamepadEx(gamepad1);
        faultPad = new GamepadEx(gamepad2);

        // --- MAPEO DE CONTROLES PARA TUNING ---

        // 1. SHOOTER: one low-power pulse per INIT. Gamepad1 A must remain held.
        new GamepadButton(driverPad, GamepadKeys.Button.A)
                .whileHeld(new RunCommand(this::requestShooterCommissioningPulse, shooter))
                .whenReleased(new InstantCommand(this::releaseShooterCommissioningPulse, shooter));

        new GamepadButton(driverPad, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> finishShooterCommissioningPulse(
                        "STOPPED_MANUAL", true), shooter));

        // 2. THREE-PIECE GATE: hold-to-run request with readiness before every piece.
        new GamepadButton(driverPad, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(this::requestThreeShotBurst, kicker, intake))
                .whenReleased(new InstantCommand(
                        this::releaseThreeShotBurst, kicker, intake));

        // Commissioning-only fail-closed injection; face buttons never request shooter output.
        new GamepadButton(faultPad, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> shooter.requestFaultInjection(
                        ShooterSubsystem.FaultInjectionMode.INVALID_VOLTAGE), shooter));
        new GamepadButton(faultPad, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> shooter.requestFaultInjection(
                        ShooterSubsystem.FaultInjectionMode.ENCODER_FROZEN), shooter));
        new GamepadButton(faultPad, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> shooter.requestFaultInjection(
                        ShooterSubsystem.FaultInjectionMode.RPM_NON_FINITE), shooter));
        new GamepadButton(faultPad, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> shooter.requestFaultInjection(
                        ShooterSubsystem.FaultInjectionMode.OVERSPEED), shooter));

        // Hold-to-run turret commissioning; every release explicitly commands zero.
        new GamepadButton(faultPad, GamepadKeys.Button.DPAD_LEFT)
                .whileHeld(new RunCommand(() -> turret.requestCommissioningJog(
                        -TURRET_COMMISSIONING_POWER,
                        TURRET_COMPARISON_PULSE_MS), turret))
                .whenReleased(new InstantCommand(() -> RobotSafety.timeZeroCommand(
                        "SYSTEM_CHECK_TURRET_LEFT_RELEASE",
                        turret::releaseCommissioningJog), turret));
        new GamepadButton(faultPad, GamepadKeys.Button.DPAD_RIGHT)
                .whileHeld(new RunCommand(() -> turret.requestCommissioningJog(
                        TURRET_COMMISSIONING_POWER,
                        TURRET_COMPARISON_PULSE_MS), turret))
                .whenReleased(new InstantCommand(() -> RobotSafety.timeZeroCommand(
                        "SYSTEM_CHECK_TURRET_RIGHT_RELEASE",
                        turret::releaseCommissioningJog), turret));
        new GamepadButton(faultPad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(turret::simulateZeroLossForCommissioning, turret));

        telemetry.addLine("✅ ROBOT LISTO PARA TUNING");
        telemetry.addLine("SHOOTER: un pulso por INIT; A=hold, B=stop");
        telemetry.addLine("INTAKE: OFF en spin-up; ON automatico al iniciar rafaga");
        telemetry.addLine("Abra http://192.168.43.1:8080/dash");
        telemetry.update();
    }

    @Override
    protected void duringInitLoop() {
        shooter.publishCommissioningTelemetrySnapshot();

        long nowNanos = System.nanoTime();
        boolean confirmationHeld = gamepad2.start && gamepad2.back;
        if (turretArming.update(confirmationHeld, nowNanos)) {
            turret.confirmCenteredAndResetEncoder();
        }

        telemetry.addLine("=== TORRETA: ARMADO T4 ===");
        telemetry.addData("Turret/Arming", turretArming.getState());
        telemetry.addData("Turret/Arm progress START+BACK", "%.0f%%",
                turretArming.getProgress(nowNanos) * 100.0);
        telemetry.addData("Turret/Zero", turret.getZeroState());
        telemetry.addData("Turret/Ticks", turret.getPosition());
        telemetry.addLine("Alinee primero las marcas de cinta.");
        telemetry.addLine("Sostenga gamepad2 START+BACK por 1 s para fijar ticks=0.");
        telemetry.addLine("La torreta no se mueve durante INIT.");
    }

    @Override
    public void run() {
        // 1. IMPORTANTE: Recargar constantes del PID/FF en cada ciclo
        // Esto permite que si cambias SHOOTER_KV en el Dashboard, el motor lo sienta DE INMEDIATO.
        shooter.reloadControllersIfConstantsChanged();

        // 2. Correr Scheduler (Comandos y Periodic del Subsystem)
        super.run();
        updateThreeShotBurst();

        // 3. Telemetría del gate cerrado T8.1.
        telemetry.addLine("--- SHOOTER: T8.1 CLOSED LOOP 2000 RPM ---");
        telemetry.addData("Shooter/Target RPM", SHOOTER_COMMISSIONING_TARGET_RPM);
        telemetry.addData("Actual RPM", shooter.getActualShooterRPM());
        telemetry.addData("Shooter Enabled", shooterEnabled);
        telemetry.addData("Shooter/Health", shooter.getHealthState());
        telemetry.addData("Shooter/Fault", shooter.getFaultReason());
        telemetry.addData("Shooter/Output limit", shooter.getOutputPowerLimit());
        telemetry.addData("Shooter/Physical output allowed", shooter.isPhysicalOutputAllowed());
        telemetry.addData("Shooter/Encoder live", shooter.getEncoderPosition());
        telemetry.addData("Shooter/Pulse active", shooterPulseActive);
        telemetry.addData("Shooter/Pulse consumed", shooterPulseConsumed);
        telemetry.addData("Shooter/Last result", shooterLastResult);
        telemetry.addData("Shooter/Pulse duration", "%.1f ms", shooterPulseDurationMs);
        telemetry.addData("Shooter/Encoder ticks", "%d -> %d",
                shooterPulseStartTicks, shooterPulseEndTicks);
        telemetry.addData("Shooter/Delta ticks", shooterPulseEndTicks - shooterPulseStartTicks);
        telemetry.addData("Shooter/Peak indicated RPM", "%.1f", shooterPulsePeakAbsRpm);
        telemetry.addData("Shooter/End indicated RPM", "%.1f", shooterPulseEndIndicatedRpm);
        telemetry.addData("Shooter/Diag SDK RPM", "%.1f",
                shooter.getDiagnosticSdkShooterRPM());
        telemetry.addData("Shooter/Diag window RPM", "%.1f",
                shooter.getDiagnosticWindowShooterRPM());
        telemetry.addData("Shooter/Diag raw ticks", shooter.getDiagnosticRawTicks());
        telemetry.addData("Shooter/Diag outward ticks",
                shooter.getDiagnosticOutwardTicks());
        telemetry.addData("Shooter/Diag SDK ticks/s", "%.1f",
                shooter.getDiagnosticSdkTicksPerSecond());
        telemetry.addData("Shooter/Diag FTCLib ticks/s", "%.1f",
                shooter.getDiagnosticFtclibTicksPerSecond());
        telemetry.addData("Shooter/Encoder responded", shooterEncoderResponded);
        telemetry.addData("Shooter/Pulse battery", "%.2f -> %.2f V",
                shooterPulseStartVoltage, shooterPulseEndVoltage);
        telemetry.addData("Shooter/Pulse min battery", "%.2f V", shooterPulseMinVoltage);
        telemetry.addData("Shooter/Max ready hold", "%.1f ms", shooterMaxReadyHoldMs);
        telemetry.addData("Shooter/Current ready hold", "%.1f ms",
                shooterCurrentReadyHoldMs);
        telemetry.addData("Fault injection", shooter.getFaultInjectionMode());
        telemetry.addData("Shooter applied power", shooter.getLastAppliedPower());
        telemetry.addLine("Gamepad1 A=2000 RPM; soltar/B/Stop=0; autocorte 8 s");
        telemetry.addLine("Sostener A+RIGHT_BUMPER=ráfaga 3 piezas, máximo 3.5 s");
        telemetry.addLine("Cada pieza exige ready >=250 ms; intake ON solo en rafaga");
        telemetry.addData("Burst/Armed", shooterPulseActive
                && shooter.isReady()
                && shooterCurrentReadyHoldMs >= BURST_READY_HOLD_MS
                && !burstConsumed);
        telemetry.addData("Burst/Active", burstActive);
        telemetry.addData("Burst/Consumed", burstConsumed);
        telemetry.addData("Burst/Result", burstResult);
        telemetry.addData("Burst/Completed shots", burstCompletedShots);
        telemetry.addData("Burst/Duration", "%.1f ms", burstDurationMs);
        telemetry.addData("Burst/Kicker state", kicker.getPulseState());
        for (int index = 0; index < BURST_SHOT_COUNT; index++) {
            telemetry.addData("Burst/Shot " + (index + 1),
                    "start=%.1f ms, pre=%.1f, min=%.1f RPM",
                    burstShotStartMs[index], burstPreRpm[index], burstMinRpm[index]);
        }
        telemetry.addLine("Gamepad2: A=voltaje, B=encoder freeze, X=RPM NaN, Y=overspeed");

        telemetry.addLine("--- TORRETA: T4 HOLD-TO-RUN ---");
        telemetry.addData("Turret/Armed", turret.isArmed());
        telemetry.addData("Turret/Zero", turret.getZeroState());
        telemetry.addData("Turret/Ticks", turret.getPosition());
        telemetry.addData("Turret/Power", turret.getLastAppliedPower());
        telemetry.addData("Turret/Move active", turret.isCommissioningMoveActive());
        telemetry.addData("Turret/Last result", turret.getLastCommissioningResult());
        telemetry.addLine("Sostener DPAD_LEFT/RIGHT; autocorte 850 ms; soltar antes=stop");
        telemetry.addLine("Gamepad2 DPAD_DOWN=invalidar cero y detener");

        // Instrumentación FND-027: duración real del hold + delta de ticks por jog.
        TurretSubsystem.JogReport jog = turret.getLastJogReport();
        telemetry.addLine("--- TORRETA: ULTIMO JOG (FND-027) ---");
        if (jog == null) {
            telemetry.addData("Jog", "SIN MEDICIÓN");
        } else {
            telemetry.addData("Jog/Duración hold", "%.1f ms", jog.holdDurationMs);
            telemetry.addData("Jog/Delta ticks", jog.deltaTicks());
            telemetry.addData("Jog/Ticks", "%d -> %d", jog.startTicks, jog.endTicks);
            telemetry.addData("Jog/Potencia", "%.3f", jog.requestedPower);
            telemetry.addData("Jog/Resultado", jog.result);
            telemetry.addData("Jog/Ticks por segundo", "%.1f",
                    jog.holdDurationMs > 0 ? jog.deltaTicks() * 1000.0 / jog.holdDurationMs : 0.0);
        }
        telemetry.addData("Turret/T4 limits", "%.2f power, ticks=[%d,%d], %d ms",
                TURRET_COMMISSIONING_POWER,
                TurretSubsystem.LIMIT_LEFT,
                TurretSubsystem.LIMIT_RIGHT,
                TURRET_COMPARISON_PULSE_MS);

        // Datos extra para diagnóstico
        telemetry.addData("Error",
                SHOOTER_COMMISSIONING_TARGET_RPM - shooter.getActualShooterRPM());
        telemetry.addData("Batería", shooter.getBatteryVoltage());
        telemetry.addData("Kicker servo solicitado", kicker.isServoEnabledAtInit());
        telemetry.addData("Kicker servo disponible", kicker.isServoAvailable());
        telemetry.addData("Kicker servo activo", kicker.isServoActive());

        telemetry.update();
    }

    private void requestShooterCommissioningPulse() {
        long nowNanos = System.nanoTime();

        if (shooterPulseConsumed || shooterBlockedUntilRelease) {
            shooter.stop();
            shooterEnabled = false;
            return;
        }
        if (!shooter.isPhysicalOutputAllowed()) {
            shooterLastResult = "REJECTED_OUTPUT_DISABLED";
            shooterBlockedUntilRelease = true;
            shooter.stop();
            shooterEnabled = false;
            return;
        }
        if (!shooter.isHealthy()) {
            shooterLastResult = "REJECTED_HEALTH_NOT_READY";
            shooterBlockedUntilRelease = true;
            shooter.stop();
            shooterEnabled = false;
            return;
        }

        if (!shooterPulseActive) {
            shooterPulseActive = true;
            shooterPulseStartNanos = nowNanos;
            shooterPulseStartTicks = shooter.getEncoderPosition();
            shooterPulseEndTicks = shooterPulseStartTicks;
            shooterPulsePeakAbsRpm = Math.abs(shooter.getActualShooterRPM());
            shooterPulseStartVoltage = shooter.getBatteryVoltage();
            shooterPulseEndVoltage = shooterPulseStartVoltage;
            shooterPulseMinVoltage = shooterPulseStartVoltage;
            shooterPulseDurationMs = 0;
            shooterEncoderResponded = false;
            shooterPulseEndIndicatedRpm = shooter.getActualShooterRPM();
            shooterReadyHoldStartNanos = 0;
            shooterCurrentReadyHoldMs = 0;
            shooterMaxReadyHoldMs = 0;
            shooterLastResult = "PULSE_ACTIVE";
        }

        shooterPulsePeakAbsRpm = Math.max(
                shooterPulsePeakAbsRpm, Math.abs(shooter.getActualShooterRPM()));
        shooterPulseEndIndicatedRpm = shooter.getActualShooterRPM();
        shooterPulseEndTicks = shooter.getEncoderPosition();
        shooterEncoderResponded = shooterEncoderResponded
                || shooterPulseEndTicks != shooterPulseStartTicks;
        shooterPulseEndVoltage = shooter.getBatteryVoltage();
        shooterPulseMinVoltage = Math.min(shooterPulseMinVoltage, shooterPulseEndVoltage);
        shooterPulseDurationMs = (nowNanos - shooterPulseStartNanos) / 1_000_000.0;

        if (shooter.isReady()) {
            if (shooterReadyHoldStartNanos == 0) {
                shooterReadyHoldStartNanos = nowNanos;
            }
            shooterCurrentReadyHoldMs =
                    (nowNanos - shooterReadyHoldStartNanos) / 1_000_000.0;
            shooterMaxReadyHoldMs = Math.max(
                    shooterMaxReadyHoldMs, shooterCurrentReadyHoldMs);
        } else {
            shooterReadyHoldStartNanos = 0;
            shooterCurrentReadyHoldMs = 0;
        }

        if (shooter.isFaultLatched()) {
            finishShooterCommissioningPulse("STOPPED_HEALTH_FAULT", true);
        } else if (!shooterEncoderResponded
                && shooterPulseDurationMs >= SHOOTER_ENCODER_RESPONSE_TIMEOUT_MS) {
            shooter.reportCommissioningEncoderNoResponse();
            finishShooterCommissioningPulse("STOPPED_ENCODER_NO_RESPONSE", true);
        } else if (shooterPulseDurationMs >= SHOOTER_COMMISSIONING_PULSE_MS) {
            finishShooterCommissioningPulse("STOPPED_TIMEOUT", true);
        } else {
            shooter.setTargetRPM(SHOOTER_COMMISSIONING_TARGET_RPM);
            shooterEnabled = true;
        }
    }

    private void releaseShooterCommissioningPulse() {
        boolean wasActive = shooterPulseActive;
        finishThreeShotBurst(burstActive ? "STOPPED_SHOOTER_RELEASE" : burstResult);
        RobotSafety.timeZeroCommand("SYSTEM_CHECK_SHOOTER_RELEASE", shooter::stop);
        shooterEnabled = false;
        shooterBlockedUntilRelease = false;
        if (wasActive) {
            finishShooterCommissioningPulse("STOPPED_RELEASE", true);
        }
    }

    private void finishShooterCommissioningPulse(String result, boolean consumePulse) {
        long nowNanos = System.nanoTime();
        boolean wasActive = shooterPulseActive;
        finishThreeShotBurst(burstActive ? "STOPPED_SHOOTER" : burstResult);
        shooter.stop();
        shooterEnabled = false;
        shooterPulseActive = false;
        shooterPulseConsumed = shooterPulseConsumed || (consumePulse && wasActive);
        shooterBlockedUntilRelease = true;
        shooterLastResult = result;

        if (wasActive) {
            shooterPulseEndIndicatedRpm = shooter.getActualShooterRPM();
            shooterPulseDurationMs = (nowNanos - shooterPulseStartNanos) / 1_000_000.0;
            shooterPulseEndTicks = shooter.getEncoderPosition();
            shooterPulsePeakAbsRpm = Math.max(
                    shooterPulsePeakAbsRpm, Math.abs(shooter.getActualShooterRPM()));
            shooterPulseEndVoltage = shooter.getBatteryVoltage();
            shooterPulseMinVoltage = Math.min(shooterPulseMinVoltage, shooterPulseEndVoltage);
        }
    }

    private void requestThreeShotBurst() {
        burstRequestHeld = true;
        if (burstConsumed || burstActive) {
            return;
        }
        if (!shooterPulseActive
                || !shooter.isHealthy()
                || !shooter.isReady()
                || shooterCurrentReadyHoldMs < BURST_READY_HOLD_MS) {
            kicker.stop();
            burstResult = "WAITING_FOR_READY";
            return;
        }

        burstConsumed = true;
        burstActive = true;
        burstStartNanos = System.nanoTime();
        burstDurationMs = 0;
        burstCompletedShots = 0;
        burstResult = "BURST_ACTIVE";
        intake.intakeOn();
        startNextBurstShot(burstStartNanos);
    }

    private void releaseThreeShotBurst() {
        burstRequestHeld = false;
        if (burstActive) {
            finishThreeShotBurst("STOPPED_REQUEST_RELEASE");
            finishShooterCommissioningPulse("STOPPED_BURST_RELEASE", true);
        }
    }

    private void updateThreeShotBurst() {
        if (!burstActive) {
            return;
        }

        long nowNanos = System.nanoTime();
        burstDurationMs = (nowNanos - burstStartNanos) / 1_000_000.0;
        if (burstShotPulseActive && burstCompletedShots < BURST_SHOT_COUNT) {
            int activeIndex = burstCompletedShots;
            burstMinRpm[activeIndex] = Math.min(
                    burstMinRpm[activeIndex], shooter.getActualShooterRPM());
        }

        if (shooter.isFaultLatched()) {
            abortBurstAndShooter("STOPPED_SHOOTER_FAULT");
        } else if (shooter.getBatteryVoltage() < BURST_MIN_BATTERY_V) {
            abortBurstAndShooter("STOPPED_LOW_BATTERY");
        } else if (!shooterPulseActive) {
            finishThreeShotBurst("STOPPED_SHOOTER");
        } else if (!burstRequestHeld) {
            abortBurstAndShooter("STOPPED_REQUEST_RELEASE");
        } else if (burstDurationMs >= BURST_MAX_DURATION_MS) {
            abortBurstAndShooter("STOPPED_BURST_TIMEOUT");
        } else if (burstShotPulseActive
                && (nowNanos - burstShotStartNanos) / 1_000_000.0
                >= LowAltitudeConstants.KICKER_EXTEND_TIME_MS) {
            RobotSafety.timeZeroCommand("SYSTEM_CHECK_BURST_PIECE_STOP", kicker::stop);
            burstShotPulseActive = false;
            burstCompletedShots++;
            if (burstCompletedShots >= BURST_SHOT_COUNT) {
                finishThreeShotBurst("COMPLETED_3_OF_3");
                finishShooterCommissioningPulse("STOPPED_BURST_COMPLETE", true);
            }
        } else if (!burstShotPulseActive
                && burstCompletedShots < BURST_SHOT_COUNT
                && kicker.getPulseState() == FeederPulseStateMachine.State.IDLE
                && shooter.isReady()
                && shooterCurrentReadyHoldMs >= BURST_READY_HOLD_MS) {
            startNextBurstShot(nowNanos);
        }
    }

    private void startNextBurstShot(long nowNanos) {
        int index = burstCompletedShots;
        burstShotStartNanos = nowNanos;
        burstShotStartMs[index] = (nowNanos - burstStartNanos) / 1_000_000.0;
        burstPreRpm[index] = shooter.getActualShooterRPM();
        burstMinRpm[index] = burstPreRpm[index];
        burstShotPulseActive = true;
        kicker.kick();
    }

    private void abortBurstAndShooter(String result) {
        finishThreeShotBurst(result);
        finishShooterCommissioningPulse(result, true);
    }

    private void finishThreeShotBurst(String result) {
        if (!burstActive) {
            return;
        }
        burstDurationMs = (System.nanoTime() - burstStartNanos) / 1_000_000.0;
        RobotSafety.timeZeroCommand("SYSTEM_CHECK_BURST_STOP", kicker::stop);
        RobotSafety.timeZeroCommand("SYSTEM_CHECK_BURST_INTAKE_STOP", intake::intakeOff);
        burstShotPulseActive = false;
        burstActive = false;
        burstResult = result;
    }
}
