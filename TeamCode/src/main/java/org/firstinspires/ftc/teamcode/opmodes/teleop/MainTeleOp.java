package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.oi.SkywalkerProfile;

@TeleOp(name = "Skywalker TeleOp", group = "Competition")
public class MainTeleOp extends CommandOpMode {

    private RobotContainer
            robotContainer;

    @Override
    public void initialize() {
        // 1. Crear el perfil de control deseado
        // Aquí podrías tener un if() para seleccionar perfiles según quién maneje
        SkywalkerProfile activeProfile = new SkywalkerProfile(gamepad1, gamepad2);

        // 2. Posición inicial (0,0,0 si no vienes de autónomo, o leer storage)
        Pose2d startPose = new Pose2d(0, 0, 0);

        // 3. Instanciar el contenedor con ese perfil
        robotContainer = new RobotContainer(hardwareMap, activeProfile, startPose);

        telemetry.addLine("Skywalker Inicializado con Perfil Estándar.");
        telemetry.update();
    }
}