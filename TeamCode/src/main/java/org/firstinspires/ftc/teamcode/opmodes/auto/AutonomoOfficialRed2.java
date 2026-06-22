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

@Autonomous(name = "AutonomoOfficialRed")
@Disabled
public class AutonomoOfficialRed2 extends SafeAutonomousOpMode {

    // Subsistemas
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private ShooterHoodSubsystem hood;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;

    @Override
    public void initialize() {
        // 1. INIT HARDWARE
        Pose2d startPose = new Pose2d(-62.5, 36.4, Math.toRadians(180)); // -52 47 127
        PoseStorage.isRedAlliance = true;

        drive = new DriveSubsystem(hardwareMap, startPose, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        hood = new ShooterHoodSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        // Aseguramos que esté apagado en INIT
        intake.intakeOff();

        // 2. CONSTRUIR TRAYECTORIAS (RoadRunner 1.0)
        MecanumDrive rrDrive = drive.getMecanumDrive();

        // Path 1: Ir a disparar
        Action path1 = rrDrive.actionBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-30,25,Math.toRadians(130)),Math.toRadians(180))
                .build();

        // Path 2: Simula ir a recoger
        Action path2 = rrDrive.actionBuilder(new Pose2d(-30, 25, Math.toRadians(130)))
                .splineToLinearHeading(new Pose2d(-10,20,Math.toRadians(270)),Math.toRadians(130))
                .strafeTo(new Vector2d(-10, 70))
                .build();
        Action path3 = rrDrive.actionBuilder(new Pose2d(-10, 70, Math.toRadians(270)))
                .strafeTo(new Vector2d(-10, 45))
                .splineToLinearHeading(new Pose2d(-30,25,Math.toRadians(130)),Math.toRadians(270))
                .build();
        Action path4 = rrDrive.actionBuilder(new Pose2d(-30, 25, Math.toRadians(130)))
                .splineToLinearHeading(new Pose2d(10,25,Math.toRadians(270)),Math.toRadians(130))
                .strafeTo(new Vector2d(10, 70))
                .build();
        Action path5 = rrDrive.actionBuilder(new Pose2d(10, 70, Math.toRadians(270)))
                .strafeTo(new Vector2d(13, 45))
                .splineToLinearHeading(new Pose2d(-30,25,Math.toRadians(130)),Math.toRadians(270))
                .build();
        Action path6 = rrDrive.actionBuilder(new Pose2d(-30, 25, Math.toRadians(130)))
                .splineToLinearHeading(new Pose2d(4, 50, Math.toRadians(90)), Math.toRadians(130))
                .build();

        configureSafePark(new SequentialCommandGroup(
                new ActionCommand(path6, drive),
                new InstantCommand(() -> {
                    shooter.stop();
                    intake.intakeOff();
                    kicker.stop();
                })
        ));


        // 3. SECUENCIA MAESTRA
        schedule(new ParallelCommandGroup(

                // --- GRUPO B: SECUENCIA PRINCIPAL DE ACCIONES ---
                new SequentialCommandGroup(
                        // 1. Configuración Inicial (Hood y Shooter)

                        // NOTA: Quité el intakeOn de aquí.
                        // Solo prendemos shooter y hood.
                        new InstantCommand(() -> shooter.setTargetRPM(2900)),
                        new InstantCommand(() -> hood.setPosition(LowAltitudeConstants.HoodPosition.SHORT_SHOT), hood),
                        new InstantCommand(() -> intake.intakeOn()),
                        // 2. Moverse a posición de disparo
                        new ActionCommand(path1, drive),

                        // 3. Disparar precargadas (3 tiros)
                        new ShootBurstCommand(shooter, hood, kicker, 3,
                                LowAltitudeConstants.TargetRPM.SHORT_SHOT_RPM,
                                LowAltitudeConstants.HoodPosition.SHORT_SHOT,
                                this::requestSafePark),

                        // --- AQUÍ PRENDEMOS EL INTAKE ---
                        // Justo antes de ir a buscar (Path 2)
                        new InstantCommand(intake::intakeOn, intake),

                        // 4. Ir por Stack 1
                        new ActionCommand(path2, drive),

                        // 5. "Kick Poquito" (Acomodar pelotas en intake)
                        new InstantCommand(kicker::kick, kicker),
                        new WaitCommand(386),
                        new InstantCommand(kicker::stop, kicker),
                        new WaitCommand(300), // Esperar que bajen

                        // 6. Regresar a disparar
                        new ActionCommand(path3, drive),

                        // 7. Disparar Stack 1
                        new ShootBurstCommand(shooter, hood, kicker, 3,
                                LowAltitudeConstants.TargetRPM.SHORT_SHOT_RPM,
                                LowAltitudeConstants.HoodPosition.SHORT_SHOT,
                                this::requestSafePark),

                        // 8. Ir por Stack 2
                        new ActionCommand(path4, drive),

                        // 9. Acomodar pelotas Stack 2
                        new InstantCommand(kicker::kick, kicker),
                        new WaitCommand(386),
                        new InstantCommand(kicker::stop, kicker),
                        new InstantCommand(kicker::stop, kicker),
                        new WaitCommand(300),

                        // 10. Regresar a disparar
                        new ActionCommand(path5, drive),

                        // 11. Disparar Stack 2
                        new ShootBurstCommand(shooter, hood, kicker, 3,
                                LowAltitudeConstants.TargetRPM.SHORT_SHOT_RPM,
                                LowAltitudeConstants.HoodPosition.SHORT_SHOT,
                                this::requestSafePark),

                        // 12. Park
                        new ActionCommand(path6, drive),

                        // 13. Apagar todo al final
                        new InstantCommand(() -> {
                            shooter.stop();
                            intake.intakeOff();
                        }),

                        // Confirmación visual
                        new InstantCommand(() -> {
                            telemetry.addData("AUTO", "FINISHED");
                            telemetry.update();
                        })
                )
        ));

        telemetry.addLine("✅ AUTO INICIALIZADO - LISTO PARA START");
        telemetry.update();
    }

    @Override
    protected void afterSchedulerRun(){
        if (drive != null && drive.getMecanumDrive() != null) {
            PoseStorage.currentPose = drive.getMecanumDrive().pose;
        }
    }
}
