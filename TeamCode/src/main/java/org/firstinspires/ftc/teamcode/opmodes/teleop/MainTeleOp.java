package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.commands.PoseStorage;
import org.firstinspires.ftc.teamcode.oi.SkywalkerProfile;

@TeleOp(name = "Skywalker TeleOp (Manual)", group = "Competition")
public class MainTeleOp extends CommandOpMode {

    private RobotContainer robotContainer;

    @Override
    public void initialize() {
        // 1. Telemetría Dual (Dashboard + Celular)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 2. Crear perfil
        SkywalkerProfile activeProfile = new SkywalkerProfile(gamepad1, gamepad2);

        // 3. Recuperar pose (con seguridad por si es null)
        Pose2d startPose = (PoseStorage.currentPose != null) ? PoseStorage.currentPose : new Pose2d(0,0,0);

        // 4. Crear contenedor
        robotContainer = new RobotContainer(this, activeProfile, startPose);

        // Mensaje de inicio
        telemetry.addLine(" SKYWALKER SYSTEMS: ONLINE ");
        telemetry.addData("Init Pose", "X: %.1f, Y: %.1f, H: %.1f",
                startPose.position.x, startPose.position.y, Math.toDegrees(startPose.heading.toDouble()));
        telemetry.update();
    }

    @Override
    public void run() {
        super.run(); // Ejecuta comandos y subsistemas

        // --- MOVER ESTO AQUÍ PARA QUE SE ACTUALICE EN VIVO ---

        telemetry.addLine("----------------------------------");
        if (PoseStorage.isRedAlliance) {
            // Usamos un caracter unicode para resaltar
            telemetry.addData("ALIANZA", "🟥 ROJO (RED) 🟥");
        } else {
            telemetry.addData("ALIANZA", "🟦 AZUL (BLUE) 🟦");
        }

        // Ver si el 'Precision Mode' está activo
        if (PoseStorage.isPrecisionMode) {
            telemetry.addData("MODO", "⚠️ PRECISIÓN (LENTO) ⚠️");
        } else {
            telemetry.addData("MODO", "🚀 TURBO (NORMAL)");
        }

        telemetry.addData("Heading", "%.1f°", Math.toDegrees(robotContainer.driveSubsystem.getHeading()));

        telemetry.update(); // Enviar a pantalla
    }
}