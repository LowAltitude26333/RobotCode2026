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


@Config
@Autonomous(name = "Autonomo Goal Out", group = "Autonomo Goul Out")
public class AutonomoGoulOut extends CommandOpMode {
    // Subsistemas
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private ShooterHoodSubsystem hood;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;
    private ColorSubsystem colorSensor;

    @Override
    public void initialize() {
        // 1. INIT HARDWARE
        // Empezamos en 0,0,0 para prueba segura
        Pose2d startPose = new Pose2d(-50, 48, Math.toRadians(130));

        drive = new DriveSubsystem(hardwareMap, startPose, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        hood = new ShooterHoodSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        colorSensor = new ColorSubsystem(hardwareMap, telemetry);

        // 2. CONSTRUIR TRAYECTORIAS "DUMMY" (RoadRunner 1.0)
        MecanumDrive rrDrive = drive.getMecanumDrive();

        // Path 1: Simula ir a disparar (Se queda en 0,0 pero espera 2 segs)
        Action path1 = rrDrive.actionBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-15, 63, Math.toRadians(180)), Math.toRadians(130))
                .build();

        // 3. SECUENCIA MAESTRA
        schedule(new ParallelCommandGroup(
                //Color detect
                new ColorDetectCommand(colorSensor),

                // --- GRUPO B: SECUENCIA DE "MOVIMIENTOS" ---
                new SequentialCommandGroup(
                        // 2. Ejecutar Path 1 (Simulación de viaje)
                        new ActionCommand(path1, drive)
                )
        ));


        telemetry.addLine("Auto SAFE MODE Cargado. El robot NO se moverá de (0,0).");
        telemetry.update();
    }
}