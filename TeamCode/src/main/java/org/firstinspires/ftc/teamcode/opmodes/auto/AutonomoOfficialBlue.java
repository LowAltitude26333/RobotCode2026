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
import org.firstinspires.ftc.teamcode.commands.PoseStorage;
import org.firstinspires.ftc.teamcode.commands.auto.ShootBurstCommand;
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
        Pose2d startPose = new Pose2d(-62.5, -36.4, Math.toRadians(180));
        PoseStorage.isRedAlliance = false;

        drive = new DriveSubsystem(hardwareMap, startPose, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        hood = new ShooterHoodSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        // --- SOLUCIÓN AL PROBLEMA DE ARRANQUE (STICTION KICK) ---
        // Le damos una orden inmediata de encendido y apagado rápido para "despegar" los motores
        // Esto ocurre durante el INIT, así que al dar START ya están sueltos.
        shooter.setTargetRPM(2900);
        sleep(100); // Pequeña pausa bloqueante solo en init para asegurar que llegue la señal

        // 2. CONSTRUIR TRAYECTORIAS
        MecanumDrive rrDrive = drive.getMecanumDrive();

        Action path1 = rrDrive.actionBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-30,-25,Math.toRadians(227)), Math.toRadians(180))
                .build();

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
                .turn(Math.toRadians(90)) // Turn usa radianes o grados según versión, asumo grados si es Action
                .build();

        // 3. SECUENCIA MAESTRA
        schedule(new ParallelCommandGroup(

                // --- GRUPO A: TAREAS DE FONDO ---
                // Prender Intake Siempre
                new InstantCommand(intake::intakeOn, intake),

                // Prender Shooter desde el principio (Spin-Up Temprano)
                // Usamos el método directo del subsistema, no un PIDCommand externo
                new InstantCommand(() -> shooter.setTargetRPM(2900), shooter),

                // --- GRUPO B: SECUENCIA DE MOVIMIENTOS ---
                new SequentialCommandGroup(
                        // 1. Configurar Hood
                        new InstantCommand(() -> hood.setPosition(LowAltitudeConstants.HoodPosition.SHORT_SHOT), hood),

                        // 2. Ir a Posición de Disparo
                        new ActionCommand(path1, drive),

                        // 3. Disparar Ráfaga (El comando espera a que esté listo)
                        // Ajusta RPM si es necesario (ej. SHORT_SHOT_RPM)
                        new ShootBurstCommand(shooter, hood, kicker, 3,
                                LowAltitudeConstants.TargetRPM.SHORT_SHOT_RPM,
                                LowAltitudeConstants.HoodPosition.SHORT_SHOT),

                        // 4. Ir por Stack 1
                        new ActionCommand(path2, drive),

                        // 5. Acomodar Pelotas ("Kick Poquito")
                        new InstantCommand(kicker::kick, kicker),
                        new WaitCommand(300), // Reduje tiempo para hacerlo más rápido
                        new InstantCommand(kicker::stop, kicker),
                        new WaitCommand(300),

                        // 6. Regresar a Disparar
                        new ActionCommand(path3, drive),

                        // 7. Disparar Stack 1
                        new ShootBurstCommand(shooter, hood, kicker, 3,
                                LowAltitudeConstants.TargetRPM.SHORT_SHOT_RPM,
                                LowAltitudeConstants.HoodPosition.SHORT_SHOT),

                        // 8. Ir por Stack 2
                        new ActionCommand(path4, drive),

                        // 9. Acomodar Pelotas
                        new InstantCommand(kicker::kick, kicker),
                        new WaitCommand(300),
                        new InstantCommand(kicker::stop, kicker),
                        new WaitCommand(300),

                        // 10. Regresar a Disparar
                        new ActionCommand(path5, drive),

                        // 11. Disparar Stack 2
                        new ShootBurstCommand(shooter, hood, kicker, 3,
                                LowAltitudeConstants.TargetRPM.SHORT_SHOT_RPM,
                                LowAltitudeConstants.HoodPosition.SHORT_SHOT),

                        // 12. Estacionarse / Girar
                        new ActionCommand(path6, drive),

                        // 13. Apagar Todo
                        new InstantCommand(shooter::stop),
                        new InstantCommand(intake::intakeOff),

                        // Guardar Pose Final
                        new InstantCommand(() -> {
                            PoseStorage.currentPose = drive.getMecanumDrive().pose;
                            telemetry.addData("AUTO", "FINISHED");
                            telemetry.update();
                        })
                )
        ));
    }

    @Override
    public void run(){
        super.run();
        // Guardado continuo de pose por seguridad
        if (drive != null && drive.getMecanumDrive() != null) {
            PoseStorage.currentPose = drive.getMecanumDrive().pose;
        }
    }
}