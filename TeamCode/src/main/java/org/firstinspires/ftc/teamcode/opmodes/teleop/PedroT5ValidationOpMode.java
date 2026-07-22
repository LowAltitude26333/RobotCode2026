package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RobotSafety;
import org.firstinspires.ftc.teamcode.localization.PedroDriveAdapter;
import org.firstinspires.ftc.teamcode.localization.PoseSnapshot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Angles;

/** Gate fisico T5 aislado: Pedro es el unico owner y no se construye Road Runner. */
@TeleOp(name = "MP02 Pedro T5 Validation", group = "MP-02")
public class PedroT5ValidationOpMode extends OpMode {
    private static final double TEST_MAX_POWER = 0.50;
    private static final double DEFAULT_DISTANCE_IN = 24.0;
    private static final double LONG_DISTANCE_IN = 48.0;
    private static final long ROUTE_TIMEOUT_NANOS = 20_000_000_000L;

    private enum Route {
        FORWARD, BACKWARD, LEFT, RIGHT, TURN_CCW_360, TURN_CW_360, SQUARE, MIXED
    }

    private enum State {
        READY, RUNNING, COMPLETE, CANCELLED, TIMEOUT, E_STOP
    }

    private PedroDriveAdapter drive;
    private Route selectedRoute = Route.FORWARD;
    private State state = State.READY;
    private double selectedDistanceIn = DEFAULT_DISTANCE_IN;
    private Pose expectedPose = new Pose();
    private PoseSnapshot finalSnapshot;
    private long routeStartNanos;
    private double turnStartTotalHeading;
    private double turnDeltaRadians;
    private int completedQuarterTurns;
    private boolean previousA;
    private boolean previousX;
    private boolean emergencyStopLatched;

    @Override
    public void init() {
        RobotSafety.beginOpMode();
        Follower follower = Constants.createFollower(hardwareMap);
        drive = new PedroDriveAdapter(follower);
        drive.setMaxPower(TEST_MAX_POWER);
        drive.trySetPose(0.0, 0.0, 0.0);
        drive.stop();
        drive.update();
        RobotSafety.registerShutdown(drive::stop);
        publishTelemetry();
    }

    @Override
    public void init_loop() {
        drive.stop();
        drive.update();

        if (gamepad1.dpad_up) selectedRoute = Route.FORWARD;
        else if (gamepad1.dpad_down) selectedRoute = Route.BACKWARD;
        else if (gamepad1.dpad_left) selectedRoute = Route.LEFT;
        else if (gamepad1.dpad_right) selectedRoute = Route.RIGHT;
        else if (gamepad1.left_bumper) selectedRoute = Route.TURN_CCW_360;
        else if (gamepad1.right_bumper) selectedRoute = Route.TURN_CW_360;
        else if (gamepad1.y) selectedRoute = Route.SQUARE;
        else if (gamepad1.b) selectedRoute = Route.MIXED;

        if (gamepad1.x && !previousX) {
            selectedDistanceIn = selectedDistanceIn == DEFAULT_DISTANCE_IN
                    ? LONG_DISTANCE_IN : DEFAULT_DISTANCE_IN;
        }
        previousX = gamepad1.x;
        state = State.READY;
        publishTelemetry();
    }

    @Override
    public void start() {
        drive.stop();
        previousA = gamepad1.a;
        state = State.READY;
    }

    @Override
    public void loop() {
        if (gamepad1.back) {
            emergencyStopLatched = true;
            state = State.E_STOP;
            long requestNanos = System.nanoTime();
            drive.stop();
            RobotSafety.stopAllTimed("MP02_T5_E_STOP", requestNanos);
            requestOpModeStop();
            return;
        }

        if (gamepad1.b && state == State.RUNNING) {
            drive.stop();
            state = State.CANCELLED;
            captureFinalAndLog();
        }

        boolean aPressed = gamepad1.a && !previousA;
        previousA = gamepad1.a;
        if (aPressed && state == State.READY) {
            launchSelectedRoute();
        }

        if (state == State.RUNNING) {
            drive.update();
            if (isTurnRoute()) {
                turnDeltaRadians = drive.getTotalHeading() - turnStartTotalHeading;
            }
            if (System.nanoTime() - routeStartNanos > ROUTE_TIMEOUT_NANOS) {
                drive.stop();
                state = State.TIMEOUT;
                captureFinalAndLog();
            } else if (!drive.isBusy()) {
                if (isTurnRoute() && completedQuarterTurns < 4) {
                    launchNextQuarterTurn();
                } else {
                    if (isTurnRoute()) {
                        turnDeltaRadians = drive.getTotalHeading() - turnStartTotalHeading;
                    }
                    drive.stop();
                    state = State.COMPLETE;
                    captureFinalAndLog();
                }
            }
        }

        publishTelemetry();
    }

    private void launchSelectedRoute() {
        drive.stop();
        drive.trySetPose(0.0, 0.0, 0.0);
        drive.update();
        routeStartNanos = System.nanoTime();
        finalSnapshot = null;
        turnDeltaRadians = 0.0;
        completedQuarterTurns = 0;
        state = State.RUNNING;

        switch (selectedRoute) {
            case FORWARD:
                launchLine(selectedDistanceIn, 0.0);
                break;
            case BACKWARD:
                launchLine(-selectedDistanceIn, 0.0);
                break;
            case LEFT:
                launchLine(0.0, selectedDistanceIn);
                break;
            case RIGHT:
                launchLine(0.0, -selectedDistanceIn);
                break;
            case TURN_CCW_360:
            case TURN_CW_360:
                expectedPose = new Pose(0.0, 0.0, 0.0);
                turnStartTotalHeading = drive.getTotalHeading();
                launchNextQuarterTurn();
                break;
            case SQUARE:
                expectedPose = new Pose(0.0, 0.0, 0.0);
                drive.followPolyline(
                        new Pose(0, 0, 0),
                        new Pose(24, 0, 0),
                        new Pose(24, 24, 0),
                        new Pose(0, 24, 0),
                        new Pose(0, 0, 0));
                break;
            case MIXED:
                expectedPose = new Pose(0.0, 0.0, 0.0);
                drive.followPolyline(
                        new Pose(0, 0, 0),
                        new Pose(30, 0, 0),
                        new Pose(30, 18, Math.PI / 2.0),
                        new Pose(12, 30, Math.PI),
                        new Pose(0, 0, 0));
                break;
        }
    }

    private void launchLine(double x, double y) {
        expectedPose = new Pose(x, y, 0.0);
        Path path = new Path(new BezierLine(new Pose(0, 0, 0), expectedPose));
        path.setConstantHeadingInterpolation(0.0);
        drive.followPath(path);
    }

    private void launchNextQuarterTurn() {
        drive.turnQuarter(selectedRoute == Route.TURN_CCW_360);
        completedQuarterTurns++;
    }

    private boolean isTurnRoute() {
        return selectedRoute == Route.TURN_CCW_360 || selectedRoute == Route.TURN_CW_360;
    }

    private void captureFinalAndLog() {
        finalSnapshot = drive.getSnapshot();
        RobotLog.ii("MP02_T5", "%s state=%s x=%.3f y=%.3f hDeg=%.2f turnDeg=%.2f",
                selectedRoute, state, finalSnapshot.xInches, finalSnapshot.yInches,
                Math.toDegrees(finalSnapshot.headingRadians), Math.toDegrees(turnDeltaRadians));
    }

    private void publishTelemetry() {
        PoseSnapshot snapshot = drive.getSnapshot();
        PoseSnapshot shown = finalSnapshot == null ? snapshot : finalSnapshot;
        double xError = expectedPose.getX() - shown.xInches;
        double yError = expectedPose.getY() - shown.yInches;
        double headingErrorDeg = Math.toDegrees(Angles.normalizeRadians(
                expectedPose.getHeading() - shown.headingRadians));

        telemetry.addLine("MP-02 T5 | PEDRO OWNER UNICO | MAX 0.50");
        telemetry.addData("Ruta", selectedRoute);
        telemetry.addData("Distancia lineal", "%.0f in", selectedDistanceIn);
        telemetry.addData("Estado", state);
        telemetry.addData("Busy", drive.isBusy());
        telemetry.addData("Pose quality", shown.quality);
        telemetry.addData("X / esperado", "%.3f / %.3f in", shown.xInches, expectedPose.getX());
        telemetry.addData("Y / esperado", "%.3f / %.3f in", shown.yInches, expectedPose.getY());
        telemetry.addData("Heading / esperado", "%.2f / %.2f deg",
                Math.toDegrees(shown.headingRadians), Math.toDegrees(expectedPose.getHeading()));
        telemetry.addData("Error estimado X/Y/H", "%.3f / %.3f in / %.2f deg",
                xError, yError, headingErrorDeg);
        if (isTurnRoute()) {
            telemetry.addData("Cuartos completados", "%d/4", completedQuarterTurns);
            telemetry.addData("Giro acumulado", "%.2f deg", Math.toDegrees(turnDeltaRadians));
        }
        telemetry.addLine("INIT: dpad=F/B/L/R, LB=360CCW, RB=360CW");
        telemetry.addLine("INIT: Y=cuadrado, B=mixta, X=24/48 in");
        telemetry.addLine("START: A ejecuta una vez, B cancela, BACK=E-stop");
        telemetry.addLine("Mida el error fisico con marcas; la pose no es medicion independiente.");
        telemetry.update();
    }

    @Override
    public void stop() {
        long requestNanos = System.nanoTime();
        drive.stop();
        if (emergencyStopLatched) {
            RobotSafety.stopAll();
        } else {
            RobotSafety.stopAllTimed("MP02_T5_STOP", requestNanos);
        }
    }
}
