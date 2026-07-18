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
    public static double TEST_TARGET_RPM = 3000;
    // MP-01 fault injection is software-only. Physical output requires a reviewed code change/new APK.
    public static final boolean SHOOTER_OUTPUT_ALLOWED = false;
    // MP-01 T4: fixed low-power hold-to-run jog inside the unchanged +/-200 tick limits.
    public static final double TURRET_COMMISSIONING_POWER = 0.05;
    public static final long TURRET_COMMISSIONING_TIMEOUT_MS = 2000;

    // Subsistemas
    private ShooterSubsystem shooter;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;
    private TurretSubsystem turret;
    private TurretArmingStateMachine turretArming;

    private GamepadEx driverPad;
    private GamepadEx faultPad;
    private boolean shooterEnabled;

    @Override
    public void initialize() {
        // Combinar telemetría de Driver Station y Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Instanciar Hardware
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        turretArming = new TurretArmingStateMachine(
                LowAltitudeConstants.TurretConstants.TURRET_ARM_HOLD_MS);

        driverPad = new GamepadEx(gamepad1);
        faultPad = new GamepadEx(gamepad2);

        // --- MAPEO DE CONTROLES PARA TUNING ---

        // 1. SHOOTER: physical output remains compiled off during MP-01 fault injection.
        new GamepadButton(driverPad, GamepadKeys.Button.A)
                .whileHeld(new RunCommand(() -> {
                    if (SHOOTER_OUTPUT_ALLOWED) {
                        shooter.setTargetRPM(TEST_TARGET_RPM);
                        shooterEnabled = true;
                    } else {
                        shooter.stop();
                        shooterEnabled = false;
                    }
                }, shooter))
                .whenReleased(new InstantCommand(() -> {
                    shooterEnabled = false;
                    RobotSafety.timeZeroCommand("SYSTEM_CHECK_SHOOTER_RELEASE", shooter::stop);
                }, shooter));

        new GamepadButton(driverPad, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    shooter.stop();
                    shooterEnabled = false;
                }));

        // 2. INTAKE & KICKER (Para probar recuperación bajo carga)
        // Gatillo Derecho: Disparar (Kicker)
        new GamepadButton(driverPad, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(kicker::kick, kicker)) // Asumiendo que kick() mueve el servo
                .whenReleased(new InstantCommand(() -> RobotSafety.timeZeroCommand(
                        "SYSTEM_CHECK_KICKER_RELEASE", kicker::stop), kicker));

        // Gatillo Izquierdo: Alimentar (Intake)
        new GamepadButton(driverPad, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new RunCommand(intake::intakeOn, intake))
                .whenReleased(new InstantCommand(() -> RobotSafety.timeZeroCommand(
                        "SYSTEM_CHECK_INTAKE_RELEASE", intake::intakeOff), intake));

        // X/Y: Control Manual de Kicker/Intake Reverso si se atora algo
        new GamepadButton(driverPad, GamepadKeys.Button.Y)
                .whileHeld(new RunCommand(intake::intakeReturn, intake))
                .whenReleased(new InstantCommand(() -> RobotSafety.timeZeroCommand(
                        "SYSTEM_CHECK_INTAKE_REVERSE_RELEASE", intake::intakeOff), intake));

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
                        TURRET_COMMISSIONING_TIMEOUT_MS), turret))
                .whenReleased(new InstantCommand(() -> RobotSafety.timeZeroCommand(
                        "SYSTEM_CHECK_TURRET_LEFT_RELEASE",
                        turret::releaseCommissioningJog), turret));
        new GamepadButton(faultPad, GamepadKeys.Button.DPAD_RIGHT)
                .whileHeld(new RunCommand(() -> turret.requestCommissioningJog(
                        TURRET_COMMISSIONING_POWER,
                        TURRET_COMMISSIONING_TIMEOUT_MS), turret))
                .whenReleased(new InstantCommand(() -> RobotSafety.timeZeroCommand(
                        "SYSTEM_CHECK_TURRET_RIGHT_RELEASE",
                        turret::releaseCommissioningJog), turret));
        new GamepadButton(faultPad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(turret::simulateZeroLossForCommissioning, turret));

        telemetry.addLine("✅ ROBOT LISTO PARA TUNING");
        telemetry.addLine("MP-01: SHOOTER FÍSICO BLOQUEADO; gamepad2 A/B/X/Y sólo faults");
        telemetry.addLine("Abra http://192.168.43.1:8080/dash");
        telemetry.update();
    }

    @Override
    protected void duringInitLoop() {
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

        // 2. Actualizar target en vivo si el motor ya está andando
        // Si ya le diste a la 'A' y cambias el número en el dashboard, se actualiza solo.
        if (shooterEnabled
                && Double.compare(shooter.getTargetRPM(), TEST_TARGET_RPM) != 0) {
            shooter.setTargetRPM(TEST_TARGET_RPM);
        }

        // 3. Correr Scheduler (Comandos y Periodic del Subsystem)
        super.run();

        // 4. Telemetría Limpia para Gráficas
        // En Dashboard, verás dos líneas. Si se superponen perfectamente, tu PID es Dios.
        telemetry.addData("Target RPM", TEST_TARGET_RPM);
        telemetry.addData("Actual RPM", shooter.getActualShooterRPM());
        telemetry.addData("Shooter Enabled", shooterEnabled);
        telemetry.addData("Shooter Output Allowed", SHOOTER_OUTPUT_ALLOWED);
        telemetry.addData("Fault injection", shooter.getFaultInjectionMode());
        telemetry.addData("Shooter applied power", shooter.getLastAppliedPower());
        telemetry.addLine("Gamepad2: A=voltaje, B=encoder freeze, X=RPM NaN, Y=overspeed");

        telemetry.addLine("--- TORRETA: T4 HOLD-TO-RUN ---");
        telemetry.addData("Turret/Armed", turret.isArmed());
        telemetry.addData("Turret/Zero", turret.getZeroState());
        telemetry.addData("Turret/Ticks", turret.getPosition());
        telemetry.addData("Turret/Power", turret.getLastAppliedPower());
        telemetry.addData("Turret/Move active", turret.isCommissioningMoveActive());
        telemetry.addData("Turret/Last result", turret.getLastCommissioningResult());
        telemetry.addLine("Sostener gamepad2 DPAD_LEFT/RIGHT=mover; soltar=stop");
        telemetry.addLine("Gamepad2 DPAD_DOWN=invalidar cero y detener");
        telemetry.addData("Turret/T4 limits", "%.2f power, ticks=[%d,%d], %d ms",
                TURRET_COMMISSIONING_POWER,
                TurretSubsystem.LIMIT_LEFT,
                TurretSubsystem.LIMIT_RIGHT,
                TURRET_COMMISSIONING_TIMEOUT_MS);

        // Datos extra para diagnóstico
        telemetry.addData("Error", TEST_TARGET_RPM - shooter.getActualShooterRPM());
        telemetry.addData("Batería", shooter.getBatteryVoltage());
        telemetry.addData("Kicker servo solicitado", kicker.isServoEnabledAtInit());
        telemetry.addData("Kicker servo disponible", kicker.isServoAvailable());
        telemetry.addData("Kicker servo activo", kicker.isServoActive());

        telemetry.update();
    }
}
