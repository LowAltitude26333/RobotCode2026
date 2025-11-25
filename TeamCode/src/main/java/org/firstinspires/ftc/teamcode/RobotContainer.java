package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.oi.ControlProfile;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterHoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class RobotContainer {

    // Subsistemas Públicos (para que el Profile pueda acceder a ellos)
    // También podrías usar getters, pero public final es aceptable en FTC para simplificar.
    public final DriveSubsystem driveSubsystem;
    public final IntakeSubsystem intakeSubsystem;
    public final ShooterSubsystem shooterSubsystem;
    public final ShooterHoodSubsystem hoodSubsystem;
    public final KickerSubsystem kickerSubsystem;

    // El perfil de control actual
    private final ControlProfile controlProfile;

    /**
     * Constructor
     * @param hardwareMap El mapa de hardware de FTC
     * @param profile El perfil de control seleccionado (OI)
     * @param startPose La posición inicial del robot (útil para auto -> teleop)
     */
    public RobotContainer(HardwareMap hardwareMap, ControlProfile profile, Pose2d startPose) {
        this.controlProfile = profile;

        // 1. Inicializar Subsistemas
        driveSubsystem = new DriveSubsystem(hardwareMap, startPose);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        hoodSubsystem = new ShooterHoodSubsystem(hardwareMap);
        kickerSubsystem = new KickerSubsystem(hardwareMap);

        // 2. Configurar Comandos por Defecto
        // El chasis siempre obedecerá al perfil seleccionado
        driveSubsystem.setDefaultCommand(new MecanumDriveCommand(
                driveSubsystem,
                profile::getDriveStrafe,
                profile::getDriveForward,
                profile::getDriveTurn
        ));

        // 3. Configurar Botones
        // Le pasamos "this" (el contenedor) al perfil para que enlace los botones
        controlProfile.configureButtonBindings(this);
    }

    // Método auxiliar para resetear el scheduler si es necesario
    public void run() {
        CommandScheduler.getInstance().run();
    }
}