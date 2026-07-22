package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.PedroPathCommand;
import org.firstinspires.ftc.teamcode.commands.PoseStorage;
import org.firstinspires.ftc.teamcode.localization.PoseSnapshot;
import org.firstinspires.ftc.teamcode.opmodes.SafeAutonomousOpMode;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;

/**
 * Alineado desde el template Pedro Pathing "Leave Goal" (DEC-041) — PERO su starting
 * pose NO está confirmada, a diferencia de los otros 9 autos de este lote.
 *
 * El path original arranca en (61.000, 75.682), muy lejos de la baldosa de salida que
 * usan los otros 9 (X≈56/85.5, Y=8) — parece pensado para arrancar desde otra posición
 * o ya con el robot adelantado en la cancha, y no trae ningún heading inicial explícito
 * (usa sólo tangent heading + reversed). No se inventó un heading aquí: se dejó 0°
 * como placeholder literal, NO como valor revisado.
 *
 * Antes de correr este auto en el robot: confirmar con el equipo desde qué baldosa/pose
 * arranca en realidad y con qué heading, y actualizar START_HEADING_RADIANS_PLACEHOLDER
 * (o mover este auto a las constantes de LowAltitudeConstants.AutoConstants una vez
 * confirmado, igual que los otros 9).
 */
@Autonomous(name = "Leave Goal", group = "Autonomous")
public class LeaveGoal extends SafeAutonomousOpMode {
    private static final double START_X_INCHES_UNCONFIRMED = 61.000;
    private static final double START_Y_INCHES_UNCONFIRMED = 75.682;
    private static final double START_HEADING_RADIANS_PLACEHOLDER = 0.0;

    private PedroDriveSubsystem drive;

    @Override
    public void initialize() {
        drive = new PedroDriveSubsystem(
                hardwareMap,
                START_X_INCHES_UNCONFIRMED,
                START_Y_INCHES_UNCONFIRMED,
                START_HEADING_RADIANS_PLACEHOLDER,
                telemetry);

        PathChain mainChain = drive.pathBuilder()
                .addPath(new BezierLine(new Pose(61.000, 75.682), new Pose(61.000, 55.000)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        schedule(new PedroPathCommand(drive, mainChain));

        telemetry.addLine("ATENCION: starting pose de 'Leave Goal' SIN CONFIRMAR con el equipo.");
        telemetry.update();
    }

    @Override
    protected void afterSchedulerRun() {
        PoseSnapshot pose = drive.getPoseSnapshot();
        // Alianza sin confirmar tampoco (ver nota de clase); no se fija isRedAlliance aquí.
        PoseStorage.currentPose = new Pose2d(pose.xInches, pose.yInches, pose.headingRadians);
    }
}
