package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotSafety;
import org.firstinspires.ftc.teamcode.localization.PedroDriveAdapter;
import org.firstinspires.ftc.teamcode.localization.PoseProvider;
import org.firstinspires.ftc.teamcode.localization.PoseSnapshot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/** Owner unico Pedro del drivetrain de produccion despues del gate T5. */
public final class PedroDriveSubsystem extends SubsystemBase {
    // Candidato solicitado para produccion; pendiente de validacion fisica controlada.
    public static final double COMMISSIONING_MAX_POWER = 0.90;

    private final PedroDriveAdapter drive;
    private final Telemetry telemetry;

    public PedroDriveSubsystem(HardwareMap hardwareMap,
                               double startXInches,
                               double startYInches,
                               double startHeadingRadians,
                               Telemetry telemetry) {
        Follower follower = Constants.createFollower(hardwareMap);
        drive = new PedroDriveAdapter(follower);
        this.telemetry = telemetry;
        drive.setMaxPower(COMMISSIONING_MAX_POWER);
        drive.trySetStartingPose(startXInches, startYInches, startHeadingRadians);
        drive.stop();
        drive.update();
        RobotSafety.registerShutdown(this::stop);
    }

    /** Contrato neutral: strafe +derecha, forward +adelante, turn +CCW. */
    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }

    public void stop() {
        drive.stop();
    }

    /** Builder de paths del Follower unico, para que los autonomos armen su PathChain. */
    public PathBuilder pathBuilder() {
        return drive.pathBuilder();
    }

    /** Arranca el seguimiento de un path autonomo. Uso: {@link org.firstinspires.ftc.teamcode.commands.PedroPathCommand}. */
    public void followPath(PathChain pathChain) {
        drive.followPath(pathChain);
    }

    /** true mientras Pedro sigue todavia el path activo. */
    public boolean isBusy() {
        return drive.isBusy();
    }

    public double getHeading() {
        return drive.getSnapshot().headingRadians;
    }

    /** El frente actual pasa a ser heading cero, conservando X/Y. */
    public void resetHeading() {
        PoseSnapshot pose = drive.getSnapshot();
        drive.trySetPose(pose.xInches, pose.yInches, 0.0);
        drive.update();
    }

    public int getResetEpoch() {
        return drive.getResetEpoch();
    }

    public double getMaxPower() {
        return COMMISSIONING_MAX_POWER;
    }

    public PoseProvider getPoseProvider() {
        return drive;
    }

    public PoseSnapshot getPoseSnapshot() {
        return drive.getSnapshot();
    }

    @Override
    public void periodic() {
        drive.update();
        PoseSnapshot pose = drive.getSnapshot();
        telemetry.addData("Drive owner", "PEDRO");
        telemetry.addData("Pose", "X: %.1f, Y: %.1f, H: %.1f deg",
                pose.xInches, pose.yInches, Math.toDegrees(pose.headingRadians));
        telemetry.addData("Pose quality", pose.quality);
    }
}
