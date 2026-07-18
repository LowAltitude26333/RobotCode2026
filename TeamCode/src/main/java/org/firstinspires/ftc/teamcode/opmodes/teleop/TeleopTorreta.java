package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.safety.TurretArmingStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem; // Asegúrate de que apunte a tu clase real
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem; // Asegúrate de que apunte a tu clase real
import org.firstinspires.ftc.teamcode.opmodes.SafeCommandOpMode;

@TeleOp(name = "Skywalker Turret: SENTINEL (Solo Conectados)", group = "Competition")
@com.qualcomm.robotcore.eventloop.opmode.Disabled
public class TeleopTorreta extends SafeCommandOpMode {

    // Subsistemas que SÍ tienes conectados
    private TurretSubsystem turretSubsystem;
    private TurretArmingStateMachine turretArming;
    private IntakeSubsystem intakeSubsystem;
    private KickerSubsystem kickerSubsystem;

    // Control (Gamepad 2)
    private GamepadEx toolOp;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 1. Inicializar SÓLO los subsistemas que vas a usar
        turretSubsystem = new TurretSubsystem(hardwareMap);
        turretArming = new TurretArmingStateMachine(
                LowAltitudeConstants.TurretConstants.TURRET_ARM_HOLD_MS);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        kickerSubsystem = new KickerSubsystem(hardwareMap);

        // 2. Configurar el Gamepad 2 de forma directa e independiente
        toolOp = new GamepadEx(gamepad2);

        // --- MAPEO DIRECTO DE BOTONES (Copiado de tu SkywalkerProfile) ---

        // RB -> Intake ON
        new GamepadButton(toolOp, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(intakeSubsystem::intakeOn, intakeSubsystem));

        // LB -> Intake REVERSE
        new GamepadButton(toolOp, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(intakeSubsystem::intakeReturn, intakeSubsystem));

        // D-Pad Down -> Intake OFF
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(intakeSubsystem::intakeOff, intakeSubsystem));

        // D-Pad Up -> Kicker Manual (Mientras se sostiene)
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_UP)
                .whileHeld(new RunCommand(kickerSubsystem::kick, kickerSubsystem))
                .whenReleased(new InstantCommand(kickerSubsystem::stop, kickerSubsystem));

        // D-Pad Left -> Kicker Retract (Desatascar)
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_LEFT)
                .whileHeld(new RunCommand(kickerSubsystem::reverse, kickerSubsystem))
                .whenReleased(new InstantCommand(kickerSubsystem::stop, kickerSubsystem));

        telemetry.addLine("--- MODO AISLADO ACTIVO ---");
        telemetry.addLine("Hardware inicializado: Torreta, Intake y Kicker.");
        telemetry.addLine("Webcam retirada; torreta sin seguimiento hasta MP-03.");
        telemetry.addLine("🎮 Usa el GAMEPAD 2.");
        telemetry.update();
    }

    @Override
    protected void duringInitLoop() {
        long nowNanos = System.nanoTime();
        boolean confirmationHeld = gamepad2.start && gamepad2.back;

        if (turretArming.update(confirmationHeld, nowNanos)) {
            turretSubsystem.confirmCenteredAndResetEncoder();
        }

        telemetry.addLine("=== ARMADO SEGURO DE TORRETA ===");
        telemetry.addData("Estado", turretArming.getState());
        telemetry.addData("Progreso START+BACK", "%.0f%%",
                turretArming.getProgress(nowNanos) * 100.0);
        telemetry.addLine("Centre físicamente la torreta antes de confirmar.");
        telemetry.addLine("El software no mueve la torreta durante init.");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run(); // Mueve la torreta sola y procesa los botones del Gamepad 2

        telemetry.addLine("=== MONITOR DE SUBSISTEMAS ===");
        telemetry.addData("Torreta Pos (Ticks)", turretSubsystem.getPosition());
        telemetry.addData("Torreta Armada", turretSubsystem.isArmed());
        telemetry.addData("Torreta", "SIN VISIÓN - DETENIDA");

        telemetry.update();
    }
}
