package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class GoToPoseCommandAction extends CommandBase {

    private final MecanumDrive drive;
    private final Pose2d targetPose;
    private Action action;

    public GoToPoseCommandAction(MecanumDrive drive, Pose2d targetPose) {
        this.drive = drive;
        this.targetPose = targetPose; // <--- CORREGIDO: Usamos el argumento real
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drive.localizer.getPose();

        // Construimos la trayectoria desde la pose actual real
        action = drive.actionBuilder(currentPose)
                .strafeTo(targetPose.position)
                .turn(targetPose.heading.toDouble() - currentPose.heading.toDouble()) // Giro optimizado
                .build();
        // ADVERTENCIA: runBlocking detiene el bucle del TeleOp mientras se mueve.
        // El robot no responderá al joystick hasta llegar al destino.
        Actions.runBlocking(action);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}