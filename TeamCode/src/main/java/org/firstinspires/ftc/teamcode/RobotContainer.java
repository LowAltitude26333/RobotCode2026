package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode; // <--- Importar esto
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry; // <--- Importar esto

import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.oi.ControlProfile;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class RobotContainer {

    // Subsistemas Públicos
    public final DriveSubsystem driveSubsystem;
    public final IntakeSubsystem intakeSubsystem;
    public final ShooterSubsystem shooterSubsystem;
    public final ShooterHoodSubsystem hoodSubsystem;
    public final KickerSubsystem kickerSubsystem;

    private final ControlProfile controlProfile;

    /**
     * Constructor Actualizado
     * @param opMode Pasamos el OpMode completo para sacar hardwareMap y telemetry
     * @param profile El perfil de control seleccionado (OI)
     * @param startPose La posición inicial del robot
     */
    public RobotContainer(CommandOpMode opMode, ControlProfile profile, Pose2d startPose) { // <--- CAMBIO: Recibe CommandOpMode
        this.controlProfile = profile;

        // Extraemos las herramientas del OpMode
        HardwareMap hardwareMap = opMode.hardwareMap;
        Telemetry telemetry = opMode.telemetry;       // <--- CAMBIO: Obtenemos telemetría

        // 1. Inicializar Subsistemas (Pasando telemetría a los que la necesitan)

        // DriveSubsystem ahora pide telemetry para debuggear el Turn
        driveSubsystem = new DriveSubsystem(hardwareMap, startPose, telemetry); // <--- CAMBIO

        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        // ShooterSubsystem ahora pide telemetry para ver RPMs
        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry); // <--- CAMBIO

        // HoodSubsystem ahora pide telemetry para ver ángulos
        hoodSubsystem = new ShooterHoodSubsystem(hardwareMap, telemetry); // <--- CAMBIO

        kickerSubsystem = new KickerSubsystem(hardwareMap);

        // 2. Configurar Comandos por Defecto
        driveSubsystem.setDefaultCommand(new MecanumDriveCommand(
                driveSubsystem,
                profile::getDriveStrafe,
                profile::getDriveForward,
                profile::getDriveTurn
        ));


        // 3. Configurar Botones
        controlProfile.configureButtonBindings(this);
    }

    public void run() {
        CommandScheduler.getInstance().run();
    }
}