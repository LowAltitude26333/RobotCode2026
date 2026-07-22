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
 * Alineado desde el template Pedro Pathing "2 Artifacts Blue Full" (DEC-041): mismos
 * waypoints del export original, ahora con SafeAutonomousOpMode/E-stop/RobotSafety
 * y starting pose real en vez del placeholder (72, 8, 90°) que traían los 10 templates.
 *
 * Dispara las 2 piezas juntas al terminar el path, con el robot detenido (decisión
 * del equipo 2026-07-22). El disparo permanece como scaffolding fail-closed hasta MP-06.
 */
@Autonomous(name = "2 Artifacts Blue Full", group = "Autonomous")
public class TwoArtifactsBlueFull extends SafeAutonomousOpMode {
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

        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap, telemetry);
        KickerSubsystem kicker = new KickerSubsystem(hardwareMap);

        PathChain mainChain = drive.pathBuilder()
                .addPath(new BezierLine(new Pose(85.500, 8.000), new Pose(105.500, 35.000)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .addPath(new BezierLine(new Pose(105.500, 35.000), new Pose(126.639, 35.000)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(
                        new Pose(126.639, 35.000), new Pose(95.778, 35.882), new Pose(85.560, 7.483)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .addPath(new BezierLine(new Pose(85.560, 7.483), new Pose(102.500, 59.000)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .addPath(new BezierLine(new Pose(102.500, 59.000), new Pose(125.500, 59.000)))
                .setTangentHeadingInterpolation()
                .addPath(new BezierCurve(
                        new Pose(125.500, 59.000), new Pose(97.235, 68.702), new Pose(85.242, 8.063)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
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
        PoseStorage.recordAutonomousResult(pose.xInches, pose.yInches, pose.headingRadians, false);
    }
}
