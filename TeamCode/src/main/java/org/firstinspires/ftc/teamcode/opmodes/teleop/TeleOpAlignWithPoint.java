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

import java.util.ArrayList;
import java.util.List;

/**
 * This opmode demonstrates how to implement "drive to point" behavior in teleop using
 * Road Runner's built-in Actions system - the same system used in autonomous!
 *
 * You specify a desired vector (x/y coordinate) via `targetPosition`. In the `DRIVE_TO_POINT`
 * mode, the bot will use Road Runner's trajectory follower to autonomously drive to that point.
 *
 * Press `a` to drive to the target point and `b` to cancel and return to manual control.
 *
 * Updated for Road Runner 1.0+ (0.1.23) using Actions
 */
@TeleOp(name = "boton", group = "Competition")
public class TeleOpAlignWithPoint extends LinearOpMode {

    public static double DRAWING_TARGET_RADIUS = 2;

    // Target position for the robot to drive to
    public static double TARGET_X = 0;
    public static double TARGET_Y = 0;
    public static double TARGET_HEADING = Math.toRadians(0); // Heading in radians

    // Define 2 states, driver control or autonomous driving
    enum Mode {
        NORMAL_CONTROL,
        DRIVE_TO_POINT
    }

    private Mode currentMode = Mode.NORMAL_CONTROL;
    private Action currentAction = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize MecanumDrive
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // List to track running actions
        List<Action> runningActions = new ArrayList<>();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update pose estimate
            drive.updatePoseEstimate();

            // Read current pose
            Pose2d poseEstimate = drive.pose;

            telemetry.addData("mode", currentMode);

            // Declare telemetry packet for dashboard field drawing
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            switch (currentMode) {
                case NORMAL_CONTROL:
                    // Switch into drive-to-point mode if `a` is pressed
                    if (gamepad1.a) {
                        currentMode = Mode.DRIVE_TO_POINT;

                        // Create target pose
                        Pose2d targetPose = new Pose2d(TARGET_X, TARGET_Y, TARGET_HEADING);

                        // Build trajectory action to drive to target
                        // This uses Road Runner's built-in path following!
                        Action driveAction = drive.actionBuilder(poseEstimate)
                                .strafeToLinearHeading(
                                        new Vector2d(TARGET_X, TARGET_Y),
                                        TARGET_HEADING
                                )
                                .build();

                        // Add action to running list
                        runningActions.add(driveAction);
                    }

                    // Standard teleop control
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x
                            ),
                            -gamepad1.right_stick_x
                    ));
                    break;

                case DRIVE_TO_POINT:
                    // Switch back into normal driver control mode if `b` is pressed
                    if (gamepad1.b) {
                        currentMode = Mode.NORMAL_CONTROL;
                        // Clear all running actions
                        runningActions.clear();
                    }

                    // Draw the target on the field
                    fieldOverlay.setStroke("#dd2c00");
                    fieldOverlay.strokeCircle(TARGET_X, TARGET_Y, DRAWING_TARGET_RADIUS);

                    // Draw line to target
                    fieldOverlay.setStroke("#b89eff");
                    fieldOverlay.strokeLine(
                            TARGET_X, TARGET_Y,
                            poseEstimate.position.x, poseEstimate.position.y
                    );

                    // Calculate distance to target
                    Vector2d difference = new Vector2d(TARGET_X, TARGET_Y).minus(
                            new Vector2d(poseEstimate.position.x, poseEstimate.position.y)
                    );
                    double distance = difference.norm();

                    telemetry.addData("distance to target", "%.2f", distance);
                    telemetry.addData("actions running", runningActions.size());

                    // If action completes, return to normal control
                    if (runningActions.isEmpty()) {
                        currentMode = Mode.NORMAL_CONTROL;
                        telemetry.addData("status", "ARRIVED!");
                    }
                    break;
            }

            // Update all running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(fieldOverlay);
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            // Draw bot on canvas
            fieldOverlay.setStroke("#3F51B5");
            drawRobot(fieldOverlay, poseEstimate);

            // Send telemetry packet to dashboard
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.position.x);
            telemetry.addData("y", poseEstimate.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(poseEstimate.heading.toDouble()));
            telemetry.addData("target x", TARGET_X);
            telemetry.addData("target y", TARGET_Y);
            telemetry.addData("target heading (deg)", Math.toDegrees(TARGET_HEADING));
            telemetry.update();
        }
    }

    /**
     * Draw robot on dashboard
     */
    private void drawRobot(Canvas canvas, Pose2d pose) {
        final double ROBOT_RADIUS = 9;
        canvas.setStroke("#3F51B5");
        canvas.strokeCircle(pose.position.x, pose.position.y, ROBOT_RADIUS);

        // Draw heading line
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