package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterPIDCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
// Importa tu DriveSubsystem si lo creaste, o usa el wrapper directo aquí

import java.util.Collections;

@Autonomous(name = "Autonomo One Refactor", group = "PowerHouse")
public class AutonomoCommandsExample extends CommandOpMode {

    // Hardware
    private MecanumDrive drive;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private KickerSubsystem kicker;

    @Override
    public void initialize() {
        // 1. Inicializar Hardware y Subsistemas
        Pose2d startPose = new Pose2d(-50, 48, Math.toRadians(130));
        drive = new MecanumDrive(hardwareMap, startPose);

        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap);
        kicker = new KickerSubsystem(hardwareMap);

        // 2. Registrar subsistemas para que su periodic() corra automático
        // ESTO REEMPLAZA TUS THREADS DE TELEMETRÍA
        register(shooter, intake, kicker);

        // 3. Definir Trayectorias (Actions de RR)
        Action trayUno = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(-47, 40))
                .build();

        Action trayDos = drive.actionBuilder(new Pose2d(-47, 40, Math.toRadians(130)))
                .splineToLinearHeading(new Pose2d(-13, 25, Math.toRadians(270)), Math.toRadians(130))
                .strafeTo(new Vector2d(-13, 45))
                .build();

        Action traySalida = drive.actionBuilder(new Pose2d(-13, 45, Math.toRadians(270)))
                .strafeTo(new Vector2d(-47, 40)) // Ejemplo simplificado
                .build();

        // 4. Construir la Secuencia Maestra (CommandGroup)
        // Se lee de arriba a abajo, claro y limpio.
        schedule(new SequentialCommandGroup(

                // Paso A: Preparar disparo mientras nos movemos
                new ShooterPIDCommand(shooter, 3000), // Prender shooter
                new RunRoadRunner(trayUno),                             // Moverse al punto

                // Paso B: Disparar (con seguridad)
                new WaitUntilCommand(shooter::onTarget), // Esperar a que llegue a RPM reales
                new InstantCommand(kicker::kick, kicker),  // Patear
                new WaitCommand(500),                      // Esperar que salga la nota
                new InstantCommand(kicker::reverse, kicker), // Retraer

                // Paso C: Ir a buscar más (Intake)
                new InstantCommand(() -> {
                    shooter.stop();
                    intake.intakeOn();
                }, shooter, intake), // Apagar shooter, prender intake

                new RunRoadRunner(trayDos), // Ir a la pila de muestras

                // Paso D: Regresar y prepararse
                new InstantCommand(intake::intakeOff, intake),
                new RunRoadRunner(traySalida)
        ));

        // Telemetría extra global si la necesitas
        telemetry.addData("Estado", "Inicializado y listo");
        telemetry.update();
    }

    // Helper para envolver Actions rápido
    private class RunRoadRunner extends ActionCommand {
        public RunRoadRunner(Action action) {
            super(action, Collections.emptySet()); // O pasa tu driveSubsystem si tienes uno
        }
    }
}