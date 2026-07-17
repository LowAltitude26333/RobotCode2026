package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.commands.PoseStorage;
import org.firstinspires.ftc.teamcode.commands.TurretFollowTagCommand;
import org.firstinspires.ftc.teamcode.oi.SkywalkerProfile;
import org.firstinspires.ftc.teamcode.opmodes.SafeCommandOpMode;
import org.firstinspires.ftc.teamcode.safety.TurretArmingStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.vision.DecodeGoalTags;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Skywalker TeleOp (Manual Principal)", group = "Competition")
public class MainTeleOp extends SafeCommandOpMode {

    private RobotContainer robotContainer;

    // --- SISTEMA DE VISIÓN Y TORRETA INTEGRADOS ---
    private TurretSubsystem turretSubsystem;
    private TurretArmingStateMachine turretArming;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void initialize() {
        // 1. Telemetría Dual
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 2. Perfil de Controles
        SkywalkerProfile activeProfile = new SkywalkerProfile(gamepad1, gamepad2);

        // 3. Recuperar pose inicial
        Pose2d startPose = (PoseStorage.currentPose != null) ? PoseStorage.currentPose : new Pose2d(0,0,0);

        // 4. Inicializar Hardware de la Torreta y su Procesador AprilTag
        turretSubsystem = new TurretSubsystem(hardwareMap);
        turretArming = new TurretArmingStateMachine(
                LowAltitudeConstants.TurretConstants.TURRET_ARM_HOLD_MS);

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(DecodeGoalTags.createLibrary())
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .build();
        VisionPortal createdVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();
        visionPortal = createdVisionPortal;
        addResourceCleanup(createdVisionPortal::close);

        // 5. Crear el Contenedor Central de Hardware
        robotContainer = new RobotContainer(this, activeProfile, startPose);
        // 6. Asignar comando por defecto a la torreta
        turretSubsystem.setDefaultCommand(
                new TurretFollowTagCommand(
                        turretSubsystem, aprilTag, () -> PoseStorage.isRedAlliance)
        );

        telemetry.addLine("🚀 SKYWALKER SYSTEMS: JOYSTICKS CORREGIDOS 🚀");
        telemetry.update();
    }

    @Override
    protected void duringInitLoop() {
        long nowNanos = System.nanoTime();
        boolean confirmationHeld = gamepad2.start && gamepad2.back;

        if (turretArming.update(confirmationHeld, nowNanos)) {
            turretSubsystem.confirmCenteredAndResetEncoder();
        }

        telemetry.addLine("=== ARMADO SEGURO DE TORRETA ===");
        telemetry.addData("Estado", turretArming.getState());
        telemetry.addData("Progreso START+BACK", "%.0f%%",
                turretArming.getProgress(nowNanos) * 100.0);
        telemetry.addLine("Centre físicamente la torreta antes de confirmar.");
        telemetry.addLine("El software no mueve la torreta durante init.");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        // --- MONITOR DE TELEMETRÍA EN TIEMPO REAL ---
        telemetry.addLine("----------------------------------");

        if (PoseStorage.isRedAlliance) {
            telemetry.addData("ALIANZA", "🟥 ROJO (RED) 🟥");
        } else {
            telemetry.addData("ALIANZA", "🟦 AZUL (BLUE) 🟦");
        }

        if (PoseStorage.isPrecisionMode) {
            telemetry.addData("MODO CHASIS", "⚠️ PRECISIÓN (LENTO) ⚠️");
        } else {
            telemetry.addData("MODO CHASIS", "🚀 TURBO (NORMAL)");
        }

        telemetry.addData("Heading Chasis", "%.1f°", Math.toDegrees(robotContainer.driveSubsystem.getHeading()));

        // Diagnóstico de Joysticks para revisar si la Driver Hub los detecta
        telemetry.addLine("--- DIAGNÓSTICO JOYSTICKS GAMEPAD 1 ---");
        telemetry.addData("Stick Izq Y", -gamepad1.left_stick_y);
        telemetry.addData("Stick Izq X (Strafe)", -gamepad1.left_stick_x);
        telemetry.addData("Stick Der X (Giro)", gamepad1.right_stick_x);

        telemetry.addLine("--- SISTEMA SENTINELA ---");
        telemetry.addData("Torreta Ticks", turretSubsystem.getPosition());
        telemetry.addData("Tags Visibles", aprilTag.getDetections().size());
        telemetry.addData("Torreta Armada", turretSubsystem.isArmed());

        if (aprilTag.getDetections().isEmpty()) {
            telemetry.addData("Estado Torreta", "SIN TARGET - DETENIDA");
        } else {
            telemetry.addData("Estado Torreta", "🎯 TARGET TOTALMENTE FIJADO");
        }

        telemetry.update();
    }
}
