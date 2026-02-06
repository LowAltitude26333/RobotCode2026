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

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Config // ¡Crucial! Permite editar variables estáticas desde Dashboard
@TeleOp(name = "SYSTEM CHECK and TUNING", group = "Test")
public class SystemCheckOpMode extends CommandOpMode {

    // --- VARIABLES EDITABLES EN DASHBOARD ---
    // Edita esto en vivo en la sección "SystemCheckOpMode" del Dashboard
    public static double TEST_TARGET_RPM = 3000;
    public static double TEST_HOOD_ANGLE = 100; // Ajusta según el rango de tu servo (0 a 1)

    // Subsistemas
    private ShooterSubsystem shooter;
    private ShooterHoodSubsystem hood;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;

    private GamepadEx driverPad;

    @Override
    public void initialize() {
        // Combinar telemetría de Driver Station y Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Instanciar Hardware
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        hood = new ShooterHoodSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        driverPad = new GamepadEx(gamepad1);

        // --- MAPEO DE CONTROLES PARA TUNING ---

        // 1. SHOOTER (A = ON con valor del Dashboard, B = STOP)
        new GamepadButton(driverPad, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() -> {
                    shooter.setTargetRPM(TEST_TARGET_RPM);
                }));

        new GamepadButton(driverPad, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    shooter.stop();
                }));

        // 2. INTAKE & KICKER (Para probar recuperación bajo carga)
        // Gatillo Derecho: Disparar (Kicker)
        new GamepadButton(driverPad, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(kicker::kick, kicker)) // Asumiendo que kick() mueve el servo
                .whenReleased(new InstantCommand(kicker::stop, kicker));

        // Gatillo Izquierdo: Alimentar (Intake)
        new GamepadButton(driverPad, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new RunCommand(intake::intakeOn, intake))
                .whenReleased(new InstantCommand(intake::intakeOff, intake));

        // X/Y: Control Manual de Kicker/Intake Reverso si se atora algo
        new GamepadButton(driverPad, GamepadKeys.Button.Y)
                .whileHeld(new RunCommand(intake::intakeReturn, intake))
                .whenReleased(new InstantCommand(intake::intakeOff, intake));

        // --- COMANDO POR DEFECTO DEL HOOD ---
        // El Hood siempre seguirá la variable "TEST_HOOD_ANGLE" del Dashboard
        hood.setDefaultCommand(new RunCommand(() -> {
            hood.setAngle(TEST_HOOD_ANGLE); // Asumiendo setPosition o setAngle
        }, hood));

        telemetry.addLine("✅ ROBOT LISTO PARA TUNING");
        telemetry.addLine("Abra http://192.168.43.1:8080/dash");
        telemetry.update();
    }

    @Override
    public void run() {
        // 1. IMPORTANTE: Recargar constantes del PID/FF en cada ciclo
        // Esto permite que si cambias SHOOTER_KV en el Dashboard, el motor lo sienta DE INMEDIATO.
        shooter.reloadControllers();

        // 2. Actualizar target en vivo si el motor ya está andando
        // Si ya le diste a la 'A' y cambias el número en el dashboard, se actualiza solo.
        if (shooter.getActualShooterRPM() > 100 || shooter.getActualShooterRPM() > 0) {
            shooter.setTargetRPM(TEST_TARGET_RPM);
        }

        // 3. Correr Scheduler (Comandos y Periodic del Subsystem)
        super.run();

        // 4. Telemetría Limpia para Gráficas
        // En Dashboard, verás dos líneas. Si se superponen perfectamente, tu PID es Dios.
        telemetry.addData("Target RPM", TEST_TARGET_RPM);
        telemetry.addData("Actual RPM", shooter.getActualShooterRPM());

        // Datos extra para diagnóstico
        telemetry.addData("Error", TEST_TARGET_RPM - shooter.getActualShooterRPM());
        telemetry.addData("Batería", hardwareMap.voltageSensor.iterator().next().getVoltage());

        telemetry.update();
    }
}