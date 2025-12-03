package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.commands.auto.ShootBurstCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "Auto: Stationary Burst Only", group = "Competition")
public class SimpleBurstAuto extends CommandOpMode {

    private ShooterSubsystem shooter;
    private ShooterHoodSubsystem hood;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;

    @Override
    public void initialize() {
        // 1. INICIALIZAR HARDWARE
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        hood = new ShooterHoodSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        // --- SEGURIDAD CRÍTICA (SAFETY LOCK) ---
        // Forzamos el apagado explícito durante el INIT para evitar penalizaciones.
        // Aunque el robot debería nacer en 0, no confiamos en la suerte.
        intake.intakeOff();
        shooter.stop();
        kicker.stop();

        // Colocamos el hood en posición segura/inicial
        hood.setPosition(LowAltitudeConstants.HoodPosition.MID_FIELD);

        // 2. DEFINIR LA RUTINA
        // Todo lo que pongas en 'schedule' NO correrá hasta que le des PLAY.
        schedule(new ParallelCommandGroup(

                // --- TAREAS DE FONDO (Empiezan al dar PLAY) ---

                // HILO 1: PID del Shooter (Mantiene vivo el control)
                new RunCommand(shooter::runPID, shooter),

                // HILO 2: Intake SIEMPRE PRENDIDO (A partir de Play)
                new InstantCommand(intake::intakeOn, intake),

                // --- SECUENCIA PRINCIPAL ---

                // HILO 3: Lógica de Disparo
                new SequentialCommandGroup(
                        // Espera inicial de seguridad (opcional, ayuda a estabilizar voltaje)
                        new WaitCommand(500),

                        // RÁFAGA 1: 2 Tiros a Media Cancha (Default)
                        // Usa el constructor que asume MID_FIELD por defecto
                        new ShootBurstCommand(shooter, hood, kicker, 3),

                        // Pausa de recarga (El intake sigue corriendo aquí)
                        new WaitCommand(1000),

                        // RÁFAGA 2: 1 Tiro Lejano (Cambio de parámetros)
                       /* new ShootBurstCommand(shooter, hood, kicker, 1,
                                LowAltitudeConstants.TargetRPM.LONG_SHOT_RPM,
                                LowAltitudeConstants.HoodPosition.LONG_SHOT),*/

                        // --- FINALIZACIÓN ---
                        // Apagar todo limpiamente
                        new ParallelCommandGroup(
                                new InstantCommand(shooter::stop),
                                new InstantCommand(intake::intakeOff)
                        )
                )
        ));

        telemetry.addLine("Auto Listo.");
        telemetry.update();
    }
}