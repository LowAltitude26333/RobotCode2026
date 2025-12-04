package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ShooterPIDCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Config
@TeleOp(name = "Tuning: Shooter & Systems (Manual)", group = "Tuning")
public class ShooterTuningOpMode extends CommandOpMode {

    // --- VARIABLES EDITABLES EN DASHBOARD ---
    public static double TARGET_RPM = 4000;   // Resultados de test 1: 3750(target) para FULL COURT, REALES 3600RPM
    public static double TARGET_ANGLE = 45;   // Ángulo de 62 grados

    // Subsistemas
    private ShooterSubsystem shooter;
    private ShooterHoodSubsystem hood;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;

    private GamepadEx gamepad;

    @Override
    public void initialize() {
        // Telemetría dual (Celular + Dashboard)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Inicializar hardware (Pasando telemetry donde se requiere)
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        hood = new ShooterHoodSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        gamepad = new GamepadEx(gamepad1);

        // --- COMANDOS POR DEFECTO ---
        // El Hood siempre obedecerá al ángulo del Dashboard en vivo
        hood.setDefaultCommand(new RunCommand(() -> hood.setAngle(TARGET_ANGLE), hood));

        // El Shooter NO tiene default command (se queda quieto hasta que piques Y)


        // --- CONTROLES MANUALES COMPLETOS ---

        // 1. SHOOTER
        // Y -> Prender y Mantener (Usa el valor actual de TARGET_RPM)
        new GamepadButton(gamepad, GamepadKeys.Button.Y)
                .whenPressed(new ShooterPIDCommand(shooter, TARGET_RPM));

        // A -> Apagar (Mata al PIDCommand porque requiere el mismo subsistema)
        new GamepadButton(gamepad, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(shooter::stop, shooter));


        // 2. INTAKE
        // RB -> Intake ON (Comer)
        new GamepadButton(gamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(intake::intakeOn, intake));

        // LB -> Intake REVERSE (Escupir / Reverse)
        new GamepadButton(gamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(intake::intakeReturn, intake));

        // D-Pad Abajo -> Intake OFF (Parar)
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(intake::intakeOff, intake));


        // 3. KICKER
        // X -> Kick (Golpear hacia adelante)
        new GamepadButton(gamepad, GamepadKeys.Button.X)
                .whileHeld(new RunCommand(kicker::kick, kicker))
                .whenReleased(new InstantCommand(kicker::stop, kicker));

        // B -> Stop (Frenar motor)
        new GamepadButton(gamepad, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(kicker::stop, kicker));

        // D-Pad Arriba -> Retract (Reversa manual)
        new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(kicker::reverse, kicker));


        telemetry.addLine("--- GUÍA DE USO ---");
        telemetry.addLine("SHOOTER: Y (On) | A (Off)");
        telemetry.addLine("INTAKE: RB (In) | LB (Out) | D-Down (Off)");
        telemetry.addLine("KICKER: X (Kick)| D-Up (Back) | B (Stop)");
        telemetry.update();
    }

    @Override
    public void run() {
        // TRUCO DE TUNING EN VIVO:
        // El ShooterPIDCommand guarda el target en el subsistema al iniciarse.
        // Si tú cambias TARGET_RPM en la compu mientras el comando ya corre,
        // actualizamos el subsistema "a la fuerza" para que obedezca el nuevo valor.
        if (shooter.getTargetRPM() != 0 && shooter.getTargetRPM() != TARGET_RPM) {
            shooter.setTargetRPM(TARGET_RPM);
        }

        super.run(); // Ejecuta comandos y periodic()

        telemetry.addData(">> DASHBOARD <<", "Edita TARGET_RPM y TARGET_ANGLE en la PC");
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Target Angle", TARGET_ANGLE);
        telemetry.addData("OnTarget", shooter.onTarget());
        telemetry.update(); // Actualiza la pantalla
    }
}