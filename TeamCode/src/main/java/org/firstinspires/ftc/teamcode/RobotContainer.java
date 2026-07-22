package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode; // <--- Importar esto
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry; // <--- Importar esto

import org.firstinspires.ftc.teamcode.commands.drivetrain.FieldCentricDriveCommand;
import org.firstinspires.ftc.teamcode.oi.ControlProfile;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Collections;
import java.util.List;

public class RobotContainer {

    // Subsistemas Públicos

    public final PedroDriveSubsystem driveSubsystem;
    public final IntakeSubsystem intakeSubsystem;
    public final ShooterSubsystem shooterSubsystem;
    /** No-op compatibility shim; the physical hood has been removed. */
    @Deprecated
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
        // Cambio atomico MP-02: RobotContainer ya no construye MecanumDrive/Road Runner.
        driveSubsystem = new PedroDriveSubsystem(
                hardwareMap,
                startPose.position.x,
                startPose.position.y,
                startPose.heading.toDouble(),
                telemetry);

        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        // ShooterSubsystem ahora pide telemetry para ver RPMs
        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry); // <--- CAMBIO

        // Compatibility only: ShooterHoodSubsystem maps no hardware after MP-01.
        hoodSubsystem = new ShooterHoodSubsystem(hardwareMap, telemetry);

        kickerSubsystem = new KickerSubsystem(hardwareMap);

        // 2. Configurar Comandos por Defecto
       /*driveSubsystem.setDefaultCommand(new MecanumDriveCommand(
                driveSubsystem,
                profile::getDriveStrafe,
                profile::getDriveForward,
                profile::getDriveTurn
        ));

        */

        driveSubsystem.setDefaultCommand(new FieldCentricDriveCommand(driveSubsystem,
                profile::getDriveStrafe,
                profile::getDriveForward,
                profile::getDriveTurn));

        // 3. Configurar Botones
        controlProfile.configureButtonBindings(this);

    }

    public void run() {
        CommandScheduler.getInstance().run();
    }

    /** Legacy camera callers receive no detections because both webcams were removed. */
    @Deprecated
    public List<AprilTagDetection> getDetections() {
        return Collections.emptyList();
    }

}
