package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.ShooterCommand2;
import org.firstinspires.ftc.teamcode.subsystems.ShooterMotor;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem; // Asegúrate de que apunte a tu clase real
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem; // Asegúrate de que apunte a tu clase real
import org.firstinspires.ftc.teamcode.commands.TurretFollowTagCommand;
import org.firstinspires.ftc.teamcode.commands.PoseStorage;
import org.firstinspires.ftc.teamcode.opmodes.SafeCommandOpMode;
import org.firstinspires.ftc.teamcode.vision.DecodeGoalTags;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Skywalker Turret: SENTINEL (Solo Conectados)", group = "Competition")
public class TeleopTorreta extends SafeCommandOpMode {

    // Subsistemas que SÍ tienes conectados
    private TurretSubsystem turretSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private KickerSubsystem kickerSubsystem;

    private ShooterMotor shooterMotor;
    // Visión
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Control (Gamepad 2)
    private GamepadEx toolOp;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 1. Inicializar SÓLO los subsistemas que vas a usar
        turretSubsystem = new TurretSubsystem(hardwareMap);
        if (gamepad2.start && gamepad2.back) {
            turretSubsystem.confirmCenteredAndResetEncoder();
        }
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        kickerSubsystem = new KickerSubsystem(hardwareMap);
        shooterMotor = new ShooterMotor(hardwareMap, telemetry);

        // 2. Configurar Visión para AprilTags
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(DecodeGoalTags.createLibrary())
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .build();
        addResourceCleanup(visionPortal::close);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // 3. Asignar comando por defecto a la torreta (Tracking automático)
        turretSubsystem.setDefaultCommand(
                new TurretFollowTagCommand(
                        turretSubsystem, aprilTag, () -> PoseStorage.isRedAlliance)
        );

        // 4. Configurar el Gamepad 2 de forma directa e independiente
        toolOp = new GamepadEx(gamepad2);

        // --- MAPEO DIRECTO DE BOTONES (Copiado de tu SkywalkerProfile) ---

        // RB -> Intake ON
        new GamepadButton(toolOp, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(intakeSubsystem::intakeOn, intakeSubsystem));

        // LB -> Intake REVERSE
        new GamepadButton(toolOp, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(intakeSubsystem::intakeReturn, intakeSubsystem));

        // D-Pad Down -> Intake OFF
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(intakeSubsystem::intakeOff, intakeSubsystem));

        // D-Pad Up -> Kicker Manual (Mientras se sostiene)
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_UP)
                .whileHeld(new RunCommand(kickerSubsystem::kick, kickerSubsystem))
                .whenReleased(new InstantCommand(kickerSubsystem::stop, kickerSubsystem));

        // D-Pad Left -> Kicker Retract (Desatascar)
        new GamepadButton(toolOp, GamepadKeys.Button.DPAD_LEFT)
                .whileHeld(new RunCommand(kickerSubsystem::reverse, kickerSubsystem))
                .whenReleased(new InstantCommand(kickerSubsystem::stop, kickerSubsystem));

        schedule(new ShooterCommand2(
                shooterMotor,
                () -> gamepad2.a
        ));

        telemetry.addLine("--- MODO AISLADO ACTIVO ---");
        telemetry.addLine("Hardware inicializado: Torreta, Intake, Kicker y Cámara.");
        telemetry.addLine("🎮 Usa el GAMEPAD 2.");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run(); // Mueve la torreta sola y procesa los botones del Gamepad 2

        telemetry.addLine("=== MONITOR DE SUBSISTEMAS ===");
        telemetry.addData("Torreta Pos (Ticks)", turretSubsystem.getPosition());
        telemetry.addData("Tags en Vista", aprilTag.getDetections().size());
        telemetry.addData("Torreta Armada", turretSubsystem.isArmed());

        if (aprilTag.getDetections().isEmpty()) {
            telemetry.addData("Torreta", "SIN TARGET - DETENIDA");
        } else {
            telemetry.addData("Torreta", "🎯 TARGET FIJADO");
        }

        telemetry.update();
    }
}
