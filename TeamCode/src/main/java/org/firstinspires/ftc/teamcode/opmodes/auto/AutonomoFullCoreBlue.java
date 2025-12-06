package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

import java.util.function.Function;


@Config
@Autonomous(name = "Autonomo Large Zone Blue", group = "Autonomo Large Zone Blue")
public class AutonomoFullCoreBlue extends CommandOpMode {
    // Subsistemas
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private ShooterHoodSubsystem hood;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;
    private ColorSubsystem colorSensor;

    @Override
    public void initialize() {
        //Alianza inversion
        boolean AlianzaAzul = true;
        Function<Pose2d, Pose2d> poseMap = AlianzaAzul
                ? pose -> pose // lado rojo: no cambia nada
                : pose -> new Pose2d(
                -pose.position.x,      // reflejo en X
                pose.position.y,
                -pose.heading.toDouble() // invertir heading
        );
        // 1. INIT HARDWARE
        // Empezamos en 0,0,0 para prueba segura
        Pose2d startPose = new Pose2d(61, 10, Math.toRadians(180));

        drive = new DriveSubsystem(hardwareMap, startPose, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        hood = new ShooterHoodSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        //colorSensor = new ColorSubsystem(hardwareMap, telemetry);

        // 2. CONSTRUIR TRAYECTORIAS "DUMMY" (RoadRunner 1.0)
        MecanumDrive rrDrive = drive.getMecanumDrive();

        // Path 1: Simula ir a disparar (Se queda en 0,0 pero espera 2 segs)
        Action path1 = rrDrive.actionBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-17, 20, Math.toRadians(145)), Math.toRadians(180))
                .waitSeconds(1)
                .build();

        // Path 2: Simula ir a recoger (Se queda en 0,0)
        Action path2 = rrDrive.actionBuilder(new Pose2d(-17, 20, Math.toRadians(145)))
                .splineToLinearHeading(new Pose2d(12, 30, Math.toRadians(270)), Math.toRadians(145))
                .strafeTo(new Vector2d(12, 60))
                .strafeTo(new Vector2d(12, 50))
                .build();

        // Path 3: Simula regresar (Se queda en 0,0)
        Action path3 = rrDrive.actionBuilder(new Pose2d(12, 50, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(2, 45, Math.toRadians(180)), Math.toRadians(270))
                .strafeTo(new Vector2d(2, 55))
                .strafeTo(new Vector2d(2, 50))
                .splineToLinearHeading(new Pose2d(-17, 20, Math.toRadians(145)), Math.toRadians(180))
                .waitSeconds(1)
                .build();
        Action path4 = rrDrive.actionBuilder(new Pose2d(-17, 20, Math.toRadians(145)))
                .splineToLinearHeading(new Pose2d(35, 30, Math.toRadians(270)), Math.toRadians(145))
                .strafeTo(new Vector2d(35, 60))
                .strafeTo(new Vector2d(35, 35))
                .splineToLinearHeading(new Pose2d(-17, 20, Math.toRadians(145)), Math.toRadians(270))
                .waitSeconds(1)
                .build();
        Action path5 = rrDrive.actionBuilder(new Pose2d(-17, 20, Math.toRadians(145)))
                .splineToLinearHeading(new Pose2d(-12, 25, Math.toRadians(270)), Math.toRadians(145))
                .strafeTo(new Vector2d(-12, 53))
                .strafeTo(new Vector2d(-12, 47))
                .build();

        Action path6 = rrDrive.actionBuilder(new Pose2d(-12, 47, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(-17,20,Math.toRadians(145)),Math.toRadians(270))
                .waitSeconds(1)
                .build();



        // 3. SECUENCIA MAESTRA
        schedule(new ParallelCommandGroup(

                // --- GRUPO A: TAREAS DE FONDO (Corren todo el tiempo) ---
                // El Shooter mantendrá al target de RPM desde el inicio
                new ShooterPIDCommand(shooter, 3000),
                // El Intake estará prendido siempre
                new InstantCommand(intake::intakeOn, intake),

                //Color detect
                new ColorDetectCommand(colorSensor),

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

                        new ActionCommand(path4, drive),
                        new InstantCommand(kicker::kick, kicker),
                        new WaitCommand(150), // Golpe cortito
                        new InstantCommand(kicker::stop, kicker),

                        new ActionCommand(path5, drive),

                        new ActionCommand(path6, drive),

                        // Final: Apagar todo
                        new InstantCommand(shooter::stop),
                        new InstantCommand(intake::intakeOff)
                )
        ));


        telemetry.addLine("Auto SAFE MODE Cargado. El robot NO se moverá de (0,0).");
        telemetry.update();
    }
}