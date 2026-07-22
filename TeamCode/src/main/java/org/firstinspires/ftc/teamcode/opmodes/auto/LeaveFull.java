package org.firstinspires.ftc.teamcode.opmodes.auto;

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
 * Alineado desde el template Pedro Pathing "Leave Full" (DEC-041): sólo avanza para
 * dejar la baldosa de salida, sin anotar piezas. Arranca en el mismo punto que los
 * autos "Red" (56, 8, 90°) — el export no distingue alianza para esta ruta.
 */
@Autonomous(name = "Leave Full", group = "Autonomous")
public class LeaveFull extends SafeAutonomousOpMode {
    private PedroDriveSubsystem drive;

    @Override
    public void initialize() {
        drive = new PedroDriveSubsystem(
                hardwareMap,
                LowAltitudeConstants.AutoConstants.RED_START_X_INCHES,
                LowAltitudeConstants.AutoConstants.RED_START_Y_INCHES,
                LowAltitudeConstants.AutoConstants.RED_START_HEADING_RADIANS,
                telemetry);

        // El export no distingue alianza para esta ruta, pero el waypoint de arranque
        // (X=56) sólo es válido físicamente parado en la baldosa Red.
        PoseStorage.isRedAlliance = true;

        PathChain mainChain = drive.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, 36.000)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        schedule(new PedroPathCommand(drive, mainChain));
    }

    @Override
    protected void afterSchedulerRun() {
        PoseSnapshot pose = drive.getPoseSnapshot();
        PoseStorage.recordAutonomousResult(pose.xInches, pose.yInches, pose.headingRadians, true);
    }
}
