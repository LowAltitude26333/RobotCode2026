package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.commands.PedroPathCommand;
import org.firstinspires.ftc.teamcode.commands.PoseStorage;
import org.firstinspires.ftc.teamcode.localization.PoseSnapshot;
import org.firstinspires.ftc.teamcode.opmodes.SafeAutonomousOpMode;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;

/**
 * Alineado desde el template Pedro Pathing "2 Artifacts Blue Goal" (DEC-041): mismos
 * waypoints del export original, ahora con SafeAutonomousOpMode/E-stop/RobotSafety
 * y starting pose real en vez del placeholder (72, 8, 90°) que traían los 10 templates.
 *
 * Pendiente de equipo: en qué segmento disparar las 2 piezas (ShootBurstCommand +
 * LowAltitudeConstants.TargetRPM) — no se inventa ese timing aquí.
 */
@Autonomous(name = "2 Artifacts Blue Goal", group = "Autonomous")
public class TwoArtifactsBlueGoal extends SafeAutonomousOpMode {
    private PedroDriveSubsystem drive;

    @Override
    public void initialize() {
        drive = new PedroDriveSubsystem(
                hardwareMap,
                LowAltitudeConstants.AutoConstants.BLUE_START_X_INCHES,
                LowAltitudeConstants.AutoConstants.BLUE_START_Y_INCHES,
                LowAltitudeConstants.AutoConstants.BLUE_START_HEADING_RADIANS,
                telemetry);
        PoseStorage.isRedAlliance = false;

        PathChain mainChain = drive.pathBuilder()
                .addPath(new BezierLine(new Pose(85.500, 8.000), new Pose(105.500, 35.000)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .addPath(new BezierLine(new Pose(105.500, 35.000), new Pose(126.639, 35.000)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(
                        new Pose(126.639, 35.000), new Pose(95.778, 35.882), new Pose(80.000, 79.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .addPath(new BezierLine(new Pose(80.000, 79.000), new Pose(102.500, 59.000)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .addPath(new BezierLine(new Pose(102.500, 59.000), new Pose(125.500, 59.000)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(
                        new Pose(125.500, 59.000), new Pose(97.235, 68.702), new Pose(83.324, 82.839)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();

        schedule(new PedroPathCommand(drive, mainChain));
        // TODO(equipo): agregar ShootBurstCommand para las 2 piezas en el punto que confirmen.
    }

    @Override
    protected void afterSchedulerRun() {
        PoseSnapshot pose = drive.getPoseSnapshot();
        PoseStorage.recordAutonomousResult(pose.xInches, pose.yInches, pose.headingRadians, false);
    }
}
