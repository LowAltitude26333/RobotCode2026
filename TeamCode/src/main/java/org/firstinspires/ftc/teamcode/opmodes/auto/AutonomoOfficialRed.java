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
import org.firstinspires.ftc.teamcode.commands.ShooterPIDCommand;
import org.firstinspires.ftc.teamcode.commands.auto.ShootBurstCommand;
import org.firstinspires.ftc.teamcode.subsystems.ColorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "AutonomoOfficialRed 1")
public class AutonomoOfficialRed extends CommandOpMode {

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
        Pose2d startPose = new Pose2d(-52, 47, Math.toRadians(130));



        drive = new DriveSubsystem(hardwareMap, startPose, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        hood = new ShooterHoodSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        // colorSensor = new ColorSubsystem(hardwareMap, telemetry);

        // 2. CONSTRUIR TRAYECTORIAS "DUMMY" (RoadRunner 1.0)
        MecanumDrive rrDrive = drive.getMecanumDrive();

        // Path 1: Simula ir a disparar (Se queda en 0,0 pero espera 2 segs)
        Action path1 = rrDrive.actionBuilder(startPose)
                .strafeTo(new Vector2d(-30, 25))// No se mueve
                .turn(Math.toRadians(-3))
                .waitSeconds(3)
                .build();

        // Path 2: Simula ir a recoger (Se queda en 0,0)
        Action path2 = rrDrive.actionBuilder(new Pose2d(-30, 25, Math.toRadians(127)))
                .splineToLinearHeading(new Pose2d(-12,25,Math.toRadians(270)),Math.toRadians(130))
                .strafeTo(new Vector2d(-12, 50))
                .build();

        // Path 3: Simula regresar (Se queda en 0,0)
        Action path3 = rrDrive.actionBuilder(new Pose2d(-12, 50, Math.toRadians(270)))
                .strafeTo(new Vector2d(-12, 45))
                .splineToLinearHeading(new Pose2d(-30,25,Math.toRadians(130)),Math.toRadians(270))
                .waitSeconds(3)
                .build();
        Action path4 = rrDrive.actionBuilder(new Pose2d(-30, 25, Math.toRadians(130)))
                .splineToLinearHeading(new Pose2d(13,30,Math.toRadians(270)),Math.toRadians(130))
                .strafeTo(new Vector2d(13, 50))
                .build();
        Action path5 = rrDrive.actionBuilder(new Pose2d(13, 50, Math.toRadians(270)))
                .strafeTo(new Vector2d(13, 45))
                .splineToLinearHeading(new Pose2d(-30,25,Math.toRadians(130)),Math.toRadians(270))
                .waitSeconds(3)
                .build();
        //path 6 :3
        Action path6 = rrDrive.actionBuilder(new Pose2d(-30, 25, Math.toRadians(130)))
                .splineToLinearHeading(new Pose2d(-30,25,Math.toRadians(90)),Math.toRadians(130))
                .build();



        // 3. SECUENCIA MAESTRA
        schedule(new ParallelCommandGroup(

                // --- GRUPO A: TAREAS DE FONDO (Corren todo el tiempo) ---
                // El Shooter mantendrá al target de RPM desde el inicio
                new ShooterPIDCommand(shooter, 3000),
                // El Intake estará prendido siempre
                new InstantCommand(intake::intakeOn, intake),

                //Color detect
                //new ColorDetectCommand(colorSensor),

                // --- GRUPO B: SECUENCIA DE "MOVIMIENTOS" ---
                new SequentialCommandGroup(
                        // 1. Configuración Inicial (Hood)
                        new InstantCommand(() -> hood.setPosition(LowAltitudeConstants.HoodPosition.MID_FIELD), hood),

                        // 2. Ejecutar Path 1 (Simulación de viaje)
                        new ActionCommand(path1, drive),

                        // 3. Disparar las 3 pelotas precargadas
                        // (El Shooter ya debería estar listo porque se prendió al inicio)
                        new ShootBurstCommand(shooter,hood, kicker, 3),

                        // 4. Ejecutar Path 2 (Simulación ir a recoger)
                        new ActionCommand(path2, drive),

                        // 5. "Kicker Kick Poquito" (Acomodar pelotas)
                        new InstantCommand(kicker::kick, kicker),
                        new WaitCommand(150), // Golpe cortito
                        new InstantCommand(kicker::stop, kicker),

                        // Opcional: Esperar un poco para asegurar que el intake agarre
                        new WaitCommand(500),

                        // 6. Ejecutar Path 3 (Simulación regresar)
                        new ActionCommand(path3, drive),

                        // 7. Disparar las pelotas recogidas
                        new ShootBurstCommand(shooter,hood, kicker, 3),

                        //8. Path 4
                        new ActionCommand(path4, drive),

                        //9. agarra pelotas
                        new InstantCommand(kicker::kick, kicker),
                        new WaitCommand(150), // Golpe cortito
                        new InstantCommand(kicker::stop, kicker),

                        //10. alinearse con goal
                        new ActionCommand(path5, drive),

                        //11.disparar pelotas
                        new ShootBurstCommand(shooter,hood, kicker, 3),

                        //12. path 6
                        new ActionCommand(path6 , drive),

                        // Final: Apagar todo
                        new InstantCommand(shooter::stop),
                        new InstantCommand(intake::intakeOff)
                )
        ));


        telemetry.addLine("Auto SAFE MODE Cargado. El robot NO se moverá de (0,0).");
        telemetry.update();
    }
}