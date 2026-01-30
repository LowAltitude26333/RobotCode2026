package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.PoseStorage; // clase compartida para guardar pose final

import java.util.ArrayList;
import java.util.List;

/**
 * TeleOp con integración de Road Runner Actions.
 * - Control manual con joysticks.
 * - Botones a,b,x,y para ir a 4 posiciones distintas.
 * - Arranca desde la pose final del autónomo (PoseStorage).
 */
@TeleOp(name = "TeleOpBETA 1 ROJO", group = "Competition")
public class TeleOpPositionBEta extends LinearOpMode {

    public static double DRAWING_TARGET_RADIUS = 2;

    // Definir 4 posiciones (por ahora todas en 0,0,0)
    public static Pose2d targetA = new Pose2d(-36, 33, Math.toRadians(127));
    public static Pose2d targetB = new Pose2d(-4, -6, Math.toRadians(127));
    public static Pose2d targetX = new Pose2d(-30, 25, Math.toRadians(127));
    public static Pose2d targetY = new Pose2d(46, -6, Math.toRadians(147));

    // Estados de control
    enum Mode {
        NORMAL_CONTROL,
        DRIVE_TO_POINT
    }

    private Mode currentMode = Mode.NORMAL_CONTROL;

    @Override
    public void runOpMode() throws InterruptedException {
        // Inicializar MecanumDrive con la pose final del autónomo
        MecanumDrive drive = new MecanumDrive(hardwareMap,
                PoseStorage.currentPose != null ? PoseStorage.currentPose : new Pose2d(0, 0, 0));

        List<Action> runningActions = new ArrayList<>();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.updatePoseEstimate();
            Pose2d poseEstimate = drive.pose;

            telemetry.addData("mode", currentMode);

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            switch (currentMode) {
                case NORMAL_CONTROL:
                    // Selección de posición con botones
                    if (gamepad1.a) {
                        currentMode = Mode.DRIVE_TO_POINT;
                        runningActions.add(
                                drive.actionBuilder(poseEstimate)
                                        .strafeToLinearHeading(targetA.position, targetA.heading)
                                        .build()
                        );
                    } else if (gamepad1.b) {
                        currentMode = Mode.DRIVE_TO_POINT;
                        runningActions.add(
                                drive.actionBuilder(poseEstimate)
                                        .strafeToLinearHeading(targetB.position, targetB.heading)
                                        .build()
                        );
                    } else if (gamepad1.x) {
                        currentMode = Mode.DRIVE_TO_POINT;
                        runningActions.add(
                                drive.actionBuilder(poseEstimate)
                                        .strafeToLinearHeading(targetX.position, targetX.heading)
                                        .build()
                        );
                    } else if (gamepad1.y) {
                        currentMode = Mode.DRIVE_TO_POINT;
                        runningActions.add(
                                drive.actionBuilder(poseEstimate)
                                        .strafeToLinearHeading(targetY.position, targetY.heading)
                                        .build()
                        );
                    }

                    // Control manual
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x
                            ),
                            -gamepad1.right_stick_x
                    ));
                    break;

                case DRIVE_TO_POINT:
                    // Cancelar con botón back (ejemplo)
                    if (gamepad1.back) {
                        currentMode = Mode.NORMAL_CONTROL;
                        runningActions.clear();
                    }

                    // Dibujar target (ejemplo: último seleccionado)
                    fieldOverlay.setStroke("#dd2c00");
                    fieldOverlay.strokeCircle(targetA.position.x, targetA.position.y, DRAWING_TARGET_RADIUS);

                    // Línea hacia target
                    fieldOverlay.setStroke("#b89eff");
                    fieldOverlay.strokeLine(
                            targetA.position.x, targetA.position.y,
                            poseEstimate.position.x, poseEstimate.position.y
                    );

                    telemetry.addData("actions running", runningActions.size());

                    if (runningActions.isEmpty()) {
                        currentMode = Mode.NORMAL_CONTROL;
                        telemetry.addData("status", "ARRIVED!");
                    }
                    break;
            }

            // Ejecutar acciones
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(fieldOverlay);
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            // Dibujar robot
            fieldOverlay.setStroke("#3F51B5");
            drawRobot(fieldOverlay, poseEstimate);

            // Enviar al dashboard
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            // Telemetría
            telemetry.addData("x", poseEstimate.position.x);
            telemetry.addData("y", poseEstimate.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(poseEstimate.heading.toDouble()));
            telemetry.update();
        }
    }

    private void drawRobot(Canvas canvas, Pose2d pose) {
        final double ROBOT_RADIUS = 9;
        canvas.setStroke("#3F51B5");
        canvas.strokeCircle(pose.position.x, pose.position.y, ROBOT_RADIUS);

        Vector2d halfv = new Vector2d(
                Math.cos(pose.heading.toDouble()),
                Math.sin(pose.heading.toDouble())
        ).times(ROBOT_RADIUS);

        double x1 = pose.position.x + halfv.x;
        double y1 = pose.position.y + halfv.y;
        double x2 = pose.position.x + halfv.x / 2;
        double y2 = pose.position.y + halfv.y / 2;

        canvas.strokeLine(x1, y1, x2, y2);
    }
}
