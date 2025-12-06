package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.commands.DriveToAprilTagCommand;
import org.firstinspires.ftc.teamcode.oi.SkywalkerProfile;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Skywalker TeleOp w/ April", group = "Competition")
public class TeleOpApril extends CommandOpMode {

    private RobotContainer robotContainer;

    @Override
    public void initialize() {
        // 1. TRUCO POWERHOUSE: Usar MultipleTelemetry
        // Esto hace que los datos se vean en el celular Y en la computadora (FTC Dashboard) al mismo tiempo.
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 2. Crear el perfil
        SkywalkerProfile activeProfile = new SkywalkerProfile(gamepad1, gamepad2);

        // 3. Definir pose inicial
        Pose2d startPose = new Pose2d(0, 0, 0);

        // 4. Crear el RobotContainer
        robotContainer = new RobotContainer(this, activeProfile, startPose);

        telemetry.addLine("----------------------------------");
        telemetry.addLine("   SKYWALKER SYSTEMS: ONLINE      ");
        telemetry.addLine("   Telemetría: Dashboard + Driver Station");
        telemetry.addLine("----------------------------------");
        telemetry.update();
    }

    /**
     * ESTA ES LA PARTE QUE FALTABA.
     * El CommandOpMode corre un bucle interno, pero necesitamos inyectar
     * el telemetry.update() al final de cada ciclo para ver los cambios.
     */
    @Override
    public void run() {
        if (gamepad1.a) {
            schedule(new DriveToAprilTagCommand(
                    robotContainer.driveSubsystem,
                    () -> {
                        List<AprilTagDetection> dets = robotContainer.getDetections();

                        for (AprilTagDetection det : dets) {
                            if (det.id == 20 || det.id == 24) {
                                return det;
                            }
                        }
                        return null;
                    },20,24
            ));

        }
        super.run(); // Esto ejecuta los comandos y los periodic() de los subsistemas
        telemetry.update(); // Esto envía los datos a la pantalla
    }


}