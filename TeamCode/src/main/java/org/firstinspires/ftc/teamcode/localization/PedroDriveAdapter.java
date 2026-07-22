package org.firstinspires.ftc.teamcode.localization;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathBuilder;

import org.firstinspires.ftc.teamcode.util.Angles;

import java.util.Locale;

/**
 * Frontera unica hacia Pedro para MP-02. Un solo {@link Follower} posee motores,
 * paths, pose y stop; los consumidores normales solo ven DriveAdapter/PoseProvider.
 */
public final class PedroDriveAdapter implements DriveAdapter, PoseProvider {
    private final Follower follower;
    private boolean poseEverUpdated;
    private int resetEpoch;
    private String lastDiagnostic = "POSE_NOT_UPDATED";

    public PedroDriveAdapter(Follower follower) {
        if (follower == null) {
            throw new IllegalArgumentException("follower == null");
        }
        this.follower = follower;
    }

    /** Unico update de Pedro por ciclo, tanto en manual como siguiendo paths. */
    public void update() {
        follower.update();
        poseEverUpdated = true;
    }

    @Override
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        double[] command = toPedroRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
        if (!follower.isTeleopDrive()) {
            follower.startTeleopDrive();
        }
        follower.setTeleOpDrive(command[0], command[1], command[2], true);
    }

    /**
     * Convierte el contrato neutral (+derecha, +adelante, +CCW) al de Pedro
     * (+adelante, +izquierda, +CCW), con saturacion y fail-closed no finito.
     */
    public static double[] toPedroRobotCentric(double strafeRight, double forward,
                                                double turnLeft) {
        if (!allFinite(strafeRight, forward, turnLeft)) {
            return new double[]{0.0, 0.0, 0.0};
        }
        return new double[]{clamp(forward), clamp(-strafeRight), clamp(turnLeft)};
    }

    /** Convierte velocidad de campo Pedro a velocidad robot-centrica neutral. */
    public static double[] fieldVelocityToRobot(double vxField, double vyField,
                                                 double headingRadians) {
        if (!allFinite(vxField, vyField, headingRadians)) {
            return new double[]{0.0, 0.0};
        }
        double cos = Math.cos(headingRadians);
        double sin = Math.sin(headingRadians);
        return new double[]{
                cos * vxField + sin * vyField,
                -sin * vxField + cos * vyField
        };
    }

    public void setMaxPower(double maxPower) {
        follower.setMaxPower(clamp(Math.abs(maxPower)));
    }

    /** Builder de paths del Follower unico, para autonomos que arman su propio PathChain. */
    public PathBuilder pathBuilder() {
        return follower.pathBuilder();
    }

    public void followPath(Path path) {
        follower.followPath(path, false);
    }

    public void followPath(PathChain pathChain) {
        follower.followPath(pathChain, false);
    }

    /** Sigue segmentos rectos y respeta el heading indicado en cada waypoint. */
    public void followPolyline(Pose... waypoints) {
        if (waypoints == null || waypoints.length < 2) {
            throw new IllegalArgumentException("Se requieren al menos dos waypoints");
        }
        PathBuilder builder = follower.pathBuilder();
        for (int i = 0; i < waypoints.length - 1; i++) {
            builder.addPath(new BezierLine(waypoints[i], waypoints[i + 1]))
                    .setLinearHeadingInterpolation(
                            waypoints[i].getHeading(), waypoints[i + 1].getHeading());
        }
        followPath(builder.build());
    }

    public void turnQuarter(boolean counterClockwise) {
        follower.turn(Math.PI / 2.0, counterClockwise);
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public double getTotalHeading() {
        return follower.getPoseTracker().getTotalHeading();
    }

    @Override
    public void stop() {
        if (follower.isTeleopDrive()) {
            follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
            follower.update();
        }
        follower.breakFollowing();
    }

    @Override
    public PoseSnapshot getSnapshot() {
        long now = System.nanoTime();
        if (!poseEverUpdated) {
            lastDiagnostic = "POSE_NOT_UPDATED";
            return PoseSnapshot.uninitialized(now);
        }

        Pose pose = follower.getPose();
        Vector fieldVelocity = follower.getVelocity();
        double omega = follower.getPoseTracker().getAngularVelocity();
        double[] robotVelocity = fieldVelocityToRobot(
                fieldVelocity.getXComponent(), fieldVelocity.getYComponent(), pose.getHeading());
        if (!allFinite(pose.getX(), pose.getY(), pose.getHeading(),
                robotVelocity[0], robotVelocity[1], omega)) {
            lastDiagnostic = "NON_FINITE_POSE_OR_VELOCITY";
            return PoseSnapshot.invalid(now, resetEpoch, lastDiagnostic);
        }

        lastDiagnostic = "OK";
        return PoseSnapshot.of(pose.getX(), pose.getY(), pose.getHeading(),
                robotVelocity[0], robotVelocity[1], omega, now, resetEpoch,
                PoseQuality.ODOMETRY_ONLY);
    }

    @Override
    public void setPose(double xInches, double yInches, double headingRadians) {
        trySetPose(xInches, yInches, headingRadians);
    }

    @Override
    public boolean trySetPose(double xInches, double yInches, double headingRadians) {
        return tryApplyPose(xInches, yInches, headingRadians, false);
    }

    /** Solo para inicializacion, antes de que el robot haya comenzado a moverse. */
    public boolean trySetStartingPose(double xInches, double yInches,
                                      double headingRadians) {
        return tryApplyPose(xInches, yInches, headingRadians, true);
    }

    private boolean tryApplyPose(double xInches, double yInches,
                                 double headingRadians, boolean startingPose) {
        if (!allFinite(xInches, yInches, headingRadians)) {
            lastDiagnostic = String.format(Locale.US,
                    "REJECTED_SET_POSE raw x=%s y=%s h=%s",
                    xInches, yInches, headingRadians);
            return false;
        }
        Pose pose = new Pose(xInches, yInches,
                Angles.normalizeRadians(headingRadians));
        if (startingPose) {
            follower.setStartingPose(pose);
        } else {
            // Pedro indica setPose() para correcciones durante ejecucion.
            follower.setPose(pose);
        }
        resetEpoch++;
        poseEverUpdated = false;
        lastDiagnostic = startingPose
                ? "SET_STARTING_POSE_ACCEPTED"
                : "SET_RUNTIME_POSE_ACCEPTED";
        return true;
    }

    @Override
    public int getResetEpoch() {
        return resetEpoch;
    }

    @Override
    public String getLastDiagnostic() {
        return lastDiagnostic;
    }

    private static double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }

    private static boolean allFinite(double... values) {
        for (double value : values) {
            if (!Double.isFinite(value)) {
                return false;
            }
        }
        return true;
    }
}
