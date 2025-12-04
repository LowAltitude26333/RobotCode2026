package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
// Asumo que estos existen en tu proyecto:
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;

@Config
@Autonomous(name = "Autonomo Goul Medium", group = "Autonomo Goul Medium")
public class AutonomoOne extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Inicialización de Pose y Subsystems
        Pose2d startPose = new Pose2d(-52, 50, Math.toRadians(130));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap, telemetry);
        KickerSubsystem kicker = new KickerSubsystem(hardwareMap);

        // Thread para mostrar RPM en telemetría
        Thread mostrarRPMThread = new Thread(() -> {
            try {
                while (!isStopRequested()) {
                    if (opModeIsActive()) {
                        telemetry.addData("ShooterRPM", shooter.getShooterRPM());
                        telemetry.update();
                    }
                    Thread.sleep(80); // Frecuencia de actualización
                }
            } catch (InterruptedException ignored) {}
        });
        mostrarRPMThread.setDaemon(true);

        // --- Definición de Trayectorias (Actions) ---

        Action adelante = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(-47, 40))
                .waitSeconds(3)
                .build();

        Action lado = drive.actionBuilder(new Pose2d(-47, 40, Math.toRadians(130)))
                .strafeTo(new Vector2d(-13, 25))
                .turn(Math.toRadians(-220))
                .lineToY(45)
                .waitSeconds(2)
                .splineToConstantHeading(new Vector2d(-47, 40), Math.toRadians(90))
                .turn(Math.toRadians(-150))
                .build();

        /* Action turn = drive.actionBuilder(new Pose2d(-13, 25, Math.toRadians(90)))
                .lineToY(45)
                .build(); */

        /* Action atras = drive.actionBuilder(new Pose2d(28, 45, Math.toRadians(180)))
                .strafeTo(new Vector2d(58, 45))
                .build(); */

        // Corregido: faltaba cerrar el builder
        Action izquerda = drive.actionBuilder(new Pose2d(58, 45, Math.toRadians(180)))
                .strafeTo(new Vector2d(58, 15))
                .build();

        // Corregido: eliminado el artefacto de git (>>>> Stashed changes)
        Action cuatro = drive.actionBuilder(new Pose2d(-47, 40, Math.toRadians(130)))
                .splineToLinearHeading(new Pose2d(13, 30, Math.toRadians(270)), Math.toRadians(130))
                .build();

        // Acción auxiliar para telemetría dentro de SequentialAction
        Action mostrarRPM = (tp) -> {
            telemetry.addData("ShooterRPM", shooter.getShooterRPM());
            telemetry.update();
            return false; // Retorna false para indicar que la acción terminó instantáneamente
        };

        // NOTA: 'uno' y 'dos' no estaban definidos en tu código original.
        // He creado placeholders vacíos para que no marque error,
        // debes definirlos con drive.actionBuilder si los necesitas.
        Action uno = new SleepAction(0.1); 
        Action dos = new SleepAction(0.1);
        Action tres = new SleepAction(0.1);

        /*
        Action unoConIntake = new SequentialAction(
                new ParallelAction(
                        dos,
                        intake.soltar()
                ),
                intake.off()
        );
        */

        Action unoConTodo = new SequentialAction(
                shooter.setRPMAutonomous(3000),
                new ParallelAction(
                        // termina si llega a RPM
                        shooter.waitUntilTargetRPMAutonomous(),
                        /* o si pasan 5 segundos
                        new SleepAction(5.0),
                        */
                        // pero mientras tanto corre PID
                        shooter.runPIDAutonomous(),
                        mostrarRPM
                ),
                mostrarRPM,
                uno // 'uno' debe estar definido arriba
        );

        /*
        Action active = new SequentialAction(
                new ParallelAction(
                        kicker.cargar(),
                        intake.soltar()
                ),
                new SleepAction(3.0),
                intake.off(),
                kicker.off()
        );
        */

        // --- Loop de espera antes de Start ---
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Listo para iniciar: presiona PLAY");
            telemetry.update();
        }

        waitForStart();

        // Iniciar el thread de telemetría
        mostrarRPMThread.start();

        if (opModeIsActive()) {
            // Ejecución de acciones
            Actions.runBlocking(adelante);
            Actions.runBlocking(lado);
            
            // Asegúrate que 'unoConTodo' no use variables nulas
            Actions.runBlocking(unoConTodo);
            
            // Comentado porque 'unoConIntake' estaba comentado arriba
            // Actions.runBlocking(unoConIntake);
            
            // Comentado porque 'active' estaba comentado arriba
            // Actions.runBlocking(active);
            
            Actions.runBlocking(tres);
            Actions.runBlocking(cuatro);

            if (isStopRequested()) return;
            
            telemetry.addLine("Trayectoria completada");
            telemetry.update();
        }
    }
}