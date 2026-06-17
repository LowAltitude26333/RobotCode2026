package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.commands.PoseStorage;
import org.firstinspires.ftc.teamcode.commands.TurretFollowTagCommand;
import org.firstinspires.ftc.teamcode.oi.SkywalkerProfile;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Skywalker TeleOp (Manual Principal)", group = "Competition")
public class MainTeleOp extends CommandOpMode {

    private RobotContainer robotContainer;

    // --- SISTEMA DE VISIÓN Y TORRETA INTEGRADOS ---
    private TurretSubsystem turretSubsystem;
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

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();

        // 5. Crear el Contenedor Central de Hardware
        robotContainer = new RobotContainer(this, activeProfile, startPose);
// =====================================================================
        // 🔥 CONTROL DIRECTO POR JOYSTICK CORREGIDO (Ejes alineados) 🔥
        // =====================================================================
        robotContainer.driveSubsystem.setDefaultCommand(
                new RunCommand(() -> {
                    // Intercambiamos los ejes para que adelante/atrás sea avance y lados sea strafe
                    double y = -gamepad1.left_stick_x; // Antes era left_stick_y
                    double x = gamepad1.left_stick_y; // Antes era left_stick_x

                    // Giro (Joystick derecho)
                    double rx = gamepad1.right_stick_x;

                    // Mandamos las señales corregidas al chasis Mecanum
                    robotContainer.driveSubsystem.drive(y, x, rx);

                }, robotContainer.driveSubsystem)
        );
        // =====================================================================

        // 6. Asignar comando por defecto a la torreta
        turretSubsystem.setDefaultCommand(
                new TurretFollowTagCommand(turretSubsystem, aprilTag)
        );

        telemetry.addLine("🚀 SKYWALKER SYSTEMS: JOYSTICKS CORREGIDOS 🚀");
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

        if (aprilTag.getDetections().isEmpty()) {
            telemetry.addData("Estado Torreta", "🔍 BUSCANDO APRILTAG...");
        } else {
            telemetry.addData("Estado Torreta", "🎯 TARGET TOTALMENTE FIJADO");
        }

        telemetry.update();
    }
}