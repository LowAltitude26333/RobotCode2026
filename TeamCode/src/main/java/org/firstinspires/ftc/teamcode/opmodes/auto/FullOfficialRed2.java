package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.PoseStorage;
import org.firstinspires.ftc.teamcode.commands.auto.ShootBurstCommand;
import org.firstinspires.ftc.teamcode.opmodes.SafeAutonomousOpMode;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "FullOfficialRed")
@Disabled
public class FullOfficialRed2 extends SafeAutonomousOpMode {

    // Subsistemas
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private ShooterHoodSubsystem hood;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;

    @Override
    public void initialize() {
        // 1. INIT HARDWARE
        // Empezamos en la posición inicial (Blue Alliance)
        Pose2d startPose = new Pose2d(61, 10, Math.toRadians(180));
        PoseStorage.isRedAlliance = true;

        drive = new DriveSubsystem(hardwareMap, startPose, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        hood = new ShooterHoodSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        // --- CAMBIO 1: SEGURIDAD DE INIT ---
        // Forzamos el apagado inmediato por si el hardwareMap lo inicia encendido.
        intake.intakeOff();

        // --- SOLUCIÓN AL PROBLEMA DE ARRANQUE (STICTION) ---

        // 2. CONSTRUIR TRAYECTORIAS (RoadRunner 1.0)
        MecanumDrive rrDrive = drive.getMecanumDrive();

        Action path1 = rrDrive.actionBuilder(startPose)
                .splineToLinearHeading(new Pose2d(55,13,Math.toRadians(160)),Math.toRadians(180))
                .build();

        // Path 2: Simula ir a recoger (Se queda en 0,0)
        Action path2 = rrDrive.actionBuilder(new Pose2d(55, 13, Math.toRadians(160)))
                .splineToLinearHeading(new Pose2d(36,28,Math.toRadians(270)),Math.toRadians(160))
                .strafeTo(new Vector2d(36, 60))
                .build();
        Action path3 = rrDrive.actionBuilder(new Pose2d(36, 60, Math.toRadians(270)))
                .strafeTo(new Vector2d(36, 50))
                .splineToLinearHeading(new Pose2d(55,13,Math.toRadians(160)),Math.toRadians(270))
                .build();
        Action path4 = rrDrive.actionBuilder(new Pose2d(55, 13, Math.toRadians(160)))
                .splineToLinearHeading(new Pose2d(12,28,Math.toRadians(270)),Math.toRadians(160))
                .strafeTo(new Vector2d(12, 60))
                .build();
        Action path5 = rrDrive.actionBuilder(new Pose2d(12, 60, Math.toRadians(270)))
                .strafeTo(new Vector2d(12, 50))
                .splineToLinearHeading(new Pose2d(55,13,Math.toRadians(160)),Math.toRadians(270))
                .build();

        Action path6 = rrDrive.actionBuilder(new Pose2d(55, 13, Math.toRadians(160)))
                .turn(Math.toRadians(90))

                .build();


        // 3. SECUENCIA MAESTRA
        schedule(new ParallelCommandGroup(

                // --- GRUPO A: TAREAS DE FONDO (Sin requerimientos conflictivos) ---

                // Shooter prendido desde el inicio (Spin-Up Temprano)
                // ¡IMPORTANTE! Aquí quitamos ", shooter" para que no bloquee al subsistema.
                // Esto permite que ShootBurstCommand (que sí requiere shooter) corra en paralelo después.


                // --- GRUPO B: SECUENCIA PRINCIPAL DE ACCIONES ---
                new SequentialCommandGroup(
                        // 1. Configuración Inicial (Hood)
                        new InstantCommand(() -> hood.setPosition(LowAltitudeConstants.HoodPosition.SHORT_SHOT), hood),
                        new InstantCommand(() -> shooter.setTargetRPM(2900)),
                        new InstantCommand(() -> intake.intakeOn()),

                        // 2. Ejecutar Path 1 (Simulación de viaje)
                        new ActionCommand(path1, drive),

                        // 3. Disparar las 3 pelotas precargadas
                        // (El Shooter ya debería estar listo porque se prendió al inicio)

                        new ShootBurstCommand(shooter,hood, kicker, 3,
                                LowAltitudeConstants.TargetRPM.LONG_SHOT_RPM,
                                LowAltitudeConstants.HoodPosition.LONG_SHOT,
                                this::requestSafePark),

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

                        // 7. Disparar las pelot
                        // as recogidas

                        new ShootBurstCommand(shooter,hood, kicker, 3,
                                LowAltitudeConstants.TargetRPM.LONG_SHOT_RPM,
                                LowAltitudeConstants.HoodPosition.LONG_SHOT,
                                this::requestSafePark),

                        new ActionCommand(path4, drive),

                        new InstantCommand(kicker::kick, kicker),
                        new WaitCommand(485),
                        new InstantCommand(kicker::stop, kicker),

                        new WaitCommand(500),

                        new ActionCommand(path5, drive),


                        new ShootBurstCommand(shooter,hood, kicker, 3,
                                LowAltitudeConstants.TargetRPM.LONG_SHOT_RPM,
                                LowAltitudeConstants.HoodPosition.LONG_SHOT,
                                this::requestSafePark),

                        new ActionCommand(path6, drive),


                        // Final: Apagar todo
                        new InstantCommand(shooter::stop),
                        new InstantCommand(intake::intakeOff)

                )
        ));

        telemetry.addLine("✅ AUTO INICIALIZADO - LISTO PARA START");
        telemetry.update();
    }

    @Override
    protected void afterSchedulerRun(){
        // Guardar la pose actual continuamente por seguridad
        if (drive != null && drive.getMecanumDrive() != null) {
            PoseStorage.currentPose = drive.getMecanumDrive().pose;
        }
    }
}
