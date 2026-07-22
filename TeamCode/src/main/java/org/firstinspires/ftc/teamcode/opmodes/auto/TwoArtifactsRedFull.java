package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.commands.PedroPathCommand;
import org.firstinspires.ftc.teamcode.commands.PoseStorage;
import org.firstinspires.ftc.teamcode.commands.auto.ShootBurstCommand;
import org.firstinspires.ftc.teamcode.localization.PoseSnapshot;
import org.firstinspires.ftc.teamcode.opmodes.SafeAutonomousOpMode;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * Alineado desde el template Pedro Pathing "2 Artifacts Red Full" (DEC-041): mismos
 * waypoints del export original, ahora con SafeAutonomousOpMode/E-stop/RobotSafety
 * y starting pose real en vez del placeholder (72, 8, 90°) que traían los 10 templates.
 *
 * Dispara las 2 piezas juntas al terminar el path, con el robot detenido (decisión
 * del equipo 2026-07-22). El disparo permanece como scaffolding fail-closed hasta MP-06.
 */
@Autonomous(name = "2 Artifacts Red Full", group = "Autonomous")
public class TwoArtifactsRedFull extends SafeAutonomousOpMode {
    private PedroDriveSubsystem drive;

    @Override
    public void initialize() {
        drive = new PedroDriveSubsystem(
                hardwareMap,
                LowAltitudeConstants.AutoConstants.RED_START_X_INCHES,
                LowAltitudeConstants.AutoConstants.RED_START_Y_INCHES,
                LowAltitudeConstants.AutoConstants.RED_START_HEADING_RADIANS,
                telemetry);
        PoseStorage.isRedAlliance = true;

        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap, telemetry);
        KickerSubsystem kicker = new KickerSubsystem(hardwareMap);

        PathChain mainChain = drive.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(36.000, 35.000)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .addPath(new BezierLine(new Pose(36.000, 35.000), new Pose(14.861, 35.000)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(
                        new Pose(14.861, 35.000), new Pose(45.722, 35.882), new Pose(55.940, 7.483)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .addPath(new BezierLine(new Pose(55.940, 7.483), new Pose(39.000, 59.000)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                .addPath(new BezierLine(new Pose(39.000, 59.000), new Pose(16.000, 59.000)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(
                        new Pose(16.000, 59.000), new Pose(44.265, 68.702), new Pose(56.258, 8.063)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();

        schedule(new SequentialCommandGroup(
                new PedroPathCommand(drive, mainChain),
                new ShootBurstCommand(
                        shooter, kicker, 2,
                        LowAltitudeConstants.TargetRPM.LONG_SHOT_RPM,
                        drive::getPoseSnapshot)));
    }

    @Override
    protected void afterSchedulerRun() {
        PoseSnapshot pose = drive.getPoseSnapshot();
        PoseStorage.recordAutonomousResult(pose.xInches, pose.yInches, pose.headingRadians, true);
    }
}
