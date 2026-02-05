package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.ColorDetectCommand;
import org.firstinspires.ftc.teamcode.commands.PoseStorage;
import org.firstinspires.ftc.teamcode.commands.ShooterPIDCommand;
import org.firstinspires.ftc.teamcode.commands.auto.ShootBurstCommand;
import org.firstinspires.ftc.teamcode.subsystems.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "AutonomoOfficialBlue 1")
public class AutonomoOfficialBlue extends CommandOpMode {

    // Subsistemas
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private ShooterHoodSubsystem hood;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;

    @Override
    public void initialize() {
        // 1. INIT HARDWARE
        // Empezamos en 0,0,0 para prueba segura
        Pose2d startPose = new Pose2d(-62.5, -36.4, Math.toRadians(180)); // -52 -47 227
        PoseStorage.isRedAlliance = false;


        drive = new DriveSubsystem(hardwareMap, startPose, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        hood = new ShooterHoodSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        // colorSensor = new ColorSubsystem(hardwareMap, telemetry);

        // 2. CONSTRUIR TRAYECTORIAS "DUMMY" (RoadRunner 1.0)
        MecanumDrive rrDrive = drive.getMecanumDrive();
        Action path1 = rrDrive.actionBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-30,-25,Math.toRadians(227)), Math.toRadians(180))// No se mueve -30 -25
                .build();

        // Path 2: Simula ir a recoger (Se queda en 0,0)
        Action path2 = rrDrive.actionBuilder(new Pose2d(-30, -25, Math.toRadians(227)))
                .splineToLinearHeading(new Pose2d(-10,-20,Math.toRadians(90)),Math.toRadians(227))
                .strafeTo(new Vector2d(-10, -70))
                .build();
        Action path3 = rrDrive.actionBuilder(new Pose2d(-10, -70, Math.toRadians(90)))
                .strafeTo(new Vector2d(-10, -45))
                .splineToLinearHeading(new Pose2d(-30,-25,Math.toRadians(227)),Math.toRadians(90))
                .build();
        Action path4 = rrDrive.actionBuilder(new Pose2d(-30, -25, Math.toRadians(227)))
                .splineToLinearHeading(new Pose2d(10,-25,Math.toRadians(90)),Math.toRadians(227))
                .strafeTo(new Vector2d(10, -70))
                .build();
        Action path5 = rrDrive.actionBuilder(new Pose2d(10, -50, Math.toRadians(90)))
                .strafeTo(new Vector2d(10, -45))
                .splineToLinearHeading(new Pose2d(-30,-25,Math.toRadians(227)),Math.toRadians(90))
                .build();

        Action path6 = rrDrive.actionBuilder(new Pose2d(-30, -25, Math.toRadians(227)))
                .turn(90)

                .build();



        // 3. SECUENCIA MAESTRA
        schedule(new ParallelCommandGroup(

                // --- GRUPO A: TAREAS DE FONDO (Corren todo el tiempo) ---
                // El Shooter mantendrá al target de RPM desde el inicio
                new ShooterPIDCommand(shooter, 2550),
                // El Intake estará prendido siempre
                new InstantCommand(intake::intakeOn, intake),

                //Color detect
                //new ColorDetectCommand(colorSensor),

                // --- GRUPO B: SECUENCIA DE "MOVIMIENTOS" ---
                new SequentialCommandGroup(
                        // 1. Configuración Inicial (Hood)
                        new InstantCommand(() -> hood.setPosition(LowAltitudeConstants.HoodPosition.SHORT_SHOT), hood),

                        // 2. Ejecutar Path 1 (Simulación de viaje)
                        new ActionCommand(path1, drive),

                        // 3. Disparar las 3 pelotas precargadas
                        // (El Shooter ya debería estar listo porque se prendió al inicio)
                        new ShootBurstCommand(shooter,hood, kicker, 3),

                        // 4. Ejecutar Path 2 (Simulación ir a recoger)
                        new ActionCommand(path2, drive),

                        // 5. "Kicker Kick Poquito" (Acomodar pelotas)
                        new InstantCommand(kicker::kick, kicker),
                        new WaitCommand(485), // Golpe cortito
                        new InstantCommand(kicker::stop, kicker),

                        // Opcional: Esperar un poco para asegurar que el intake agarre
                        new WaitCommand(500),

                        // 6. Ejecutar Path 3 (Simulación regresar)
                        new ActionCommand(path3, drive),

                        // 7. Disparar las pelotas recogidas
                        new ShootBurstCommand(shooter,hood, kicker, 3),

                        new ActionCommand(path4, drive),

                        new InstantCommand(kicker::kick, kicker),
                        new WaitCommand(485), // Golpe cortito
                        new InstantCommand(kicker::stop, kicker),

                        new WaitCommand(500),

                        new ActionCommand(path5, drive),

                        new ShootBurstCommand(shooter,hood, kicker, 3),

                        new ActionCommand(path6, drive),


                        // Final: Apagar todo
                        new InstantCommand(shooter::stop),
                        new InstantCommand(intake::intakeOff),

                        // Final: Apagar todo
        new InstantCommand(shooter::stop),
                new InstantCommand(intake::intakeOff),

                // CONFIRMACIÓN FINAL (Opcional, ya que run() lo hace continuo)
                new InstantCommand(() -> {
                    PoseStorage.currentPose = drive.getMecanumDrive().pose;
                    telemetry.speak("Trayectoria Finalizada");
                })
                )
        ));

        telemetry.addLine(PoseStorage.currentPose.toString());
        telemetry.update();
    }
    @Override
    public void run(){
        // 1. Ejecutar el Scheduler (mantiene vivos los comandos y el drive)
        super.run();

        // 2. Guardar la pose actual en la variable estática
        // Si el match se corta al segundo 29, ¡ya tienes la posición guardada!
        if (drive != null && drive.getMecanumDrive() != null) {
            PoseStorage.currentPose = drive.getMecanumDrive().pose;
        }

        // (Opcional) Telemetría de debug para ver que funciona
        telemetry.addData("Saving Pose", "X: %.1f, Y: %.1f, H: %.1f",
                PoseStorage.currentPose.position.x,
                PoseStorage.currentPose.position.y,
                Math.toDegrees(PoseStorage.currentPose.heading.toDouble()));
        telemetry.update();
    }
}