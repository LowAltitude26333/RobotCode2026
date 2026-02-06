package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.commands.auto.ShootBurstCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Config
@Autonomous(name = "TEST: Shoot Burst Command", group = "Test")
public class TestShootBurstAuto extends CommandOpMode {

    // --- VARIABLES EDITABLES EN DASHBOARD ---
    // Modifica estos valores en http://192.168.43.1:8080/dash antes de dar Start
    public static int SHOTS_TO_FIRE = 3;
    public static LowAltitudeConstants.TargetRPM TARGET_RPM = LowAltitudeConstants.TargetRPM.LONG_SHOT_RPM;
    public static LowAltitudeConstants.HoodPosition HOOD_POS = LowAltitudeConstants.HoodPosition.LONG_SHOT;
    public static int INITIAL_DELAY_MS = 500; // Tiempo de espera antes de empezar (seguridad)

    // Subsistemas
    private ShooterSubsystem shooter;
    private ShooterHoodSubsystem hood;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;

    @Override
    public void initialize() {
        // 1. Telemetría Dual (Driver Station + Dashboard)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 2. Inicializar Hardware
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        hood = new ShooterHoodSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        // 3. Crear la Secuencia de Prueba
        // Usamos SequentialCommandGroup para asegurar el orden:
        // Espera -> Dispara Ráfaga -> Apaga Shooter
        SequentialCommandGroup testSequence = new SequentialCommandGroup(
                // A. Pequeña espera de seguridad al iniciar
                new WaitCommand(INITIAL_DELAY_MS),

                new InstantCommand(intake::intakeOn, intake),

                // B. Ejecutar tu comando de ráfaga
                // Pasamos las variables estáticas para que obedezca al Dashboard
                new ShootBurstCommand(
                        shooter,
                        hood,
                        kicker,
                        SHOTS_TO_FIRE,
                        TARGET_RPM,
                        HOOD_POS
                ),

                // C. Apagar todo al terminar (Buenas Prácticas)
                new InstantCommand(() -> {
                    shooter.stop();
                    // Opcional: hood.setPosition(LowAltitudeConstants.HoodPosition.HOME_POS.angle);
                }, shooter, hood)
        );

        // 4. Agendar el comando
        // En CommandOpMode, esto se ejecutará automáticamente cuando des "Play"
        schedule(testSequence);

        telemetry.addLine("✅ AUTO LISTO");
        telemetry.addLine("Configura SHOTS, RPM y HOOD en Dashboard.");
        telemetry.update();
    }

    @Override
    public void run() {
        // FTCLib maneja el Scheduler aquí.
        // Solo agregamos telemetría extra si es necesario.
        super.run();

        telemetry.addData("Estado", "Corriendo Auto...");
        telemetry.addData("Shooter RPM", shooter.getActualShooterRPM());
        telemetry.addData("Shooter Target", shooter.isReady() ? "READY" : "Spinning Up");
        telemetry.update();
    }
}