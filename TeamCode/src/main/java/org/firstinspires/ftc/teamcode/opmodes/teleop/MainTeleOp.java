package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.commands.PoseStorage;
import org.firstinspires.ftc.teamcode.oi.SkywalkerProfile;
import org.firstinspires.ftc.teamcode.opmodes.SafeCommandOpMode;
import org.firstinspires.ftc.teamcode.safety.TurretArmingStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp(name = "Skywalker TeleOp (Manual Principal)", group = "Competition")
public class MainTeleOp extends SafeCommandOpMode {

    private RobotContainer robotContainer;

    // Turret remains stationary until Limelight integration in MP-03.
    private TurretSubsystem turretSubsystem;
    private TurretArmingStateMachine turretArming;

    @Override
    public void initialize() {
        // 1. Telemetría Dual
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 2. Perfil de Controles
        SkywalkerProfile activeProfile = new SkywalkerProfile(gamepad1, gamepad2);

        // 3. Recuperar pose inicial
        Pose2d startPose = (PoseStorage.currentPose != null) ? PoseStorage.currentPose : new Pose2d(0,0,0);

        // 4. Inicializar torreta sin la webcam retirada.
        turretSubsystem = new TurretSubsystem(hardwareMap);
        turretArming = new TurretArmingStateMachine(
                LowAltitudeConstants.TurretConstants.TURRET_ARM_HOLD_MS);

        // 5. Crear el Contenedor Central de Hardware
        robotContainer = new RobotContainer(this, activeProfile, startPose);
        telemetry.addLine("🚀 SKYWALKER SYSTEMS: JOYSTICKS CORREGIDOS 🚀");
        telemetry.addLine("Visión anterior retirada; torreta sin seguimiento hasta MP-03.");
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
        telemetry.addData("Stick Izq X (Strafe; derecha +)", gamepad1.left_stick_x);
        telemetry.addData("Stick Der X (Giro)", gamepad1.right_stick_x);

        telemetry.addLine("--- SISTEMA SENTINELA ---");
        telemetry.addData("Torreta Ticks", turretSubsystem.getPosition());
        telemetry.addData("Torreta Armada", turretSubsystem.isArmed());
        telemetry.addData("Estado Torreta", "SIN VISIÓN - DETENIDA");

        telemetry.addLine("--- KICKER SERVO OPCIONAL ---");
        telemetry.addData("Solicitado al INIT", robotContainer.kickerSubsystem.isServoEnabledAtInit());
        telemetry.addData("Disponible", robotContainer.kickerSubsystem.isServoAvailable());
        telemetry.addData("Activo", robotContainer.kickerSubsystem.isServoActive());

        telemetry.update();
    }
}
