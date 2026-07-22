package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.RobotSafety;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * MP-02 first gate: reads Pedro localization while the robot is moved by hand.
 * This OpMode never enables teleop drive or path following and commands drivetrain
 * zero before and after every follower update.
 */
@TeleOp(name = "MP02 Pedro Localization Push Test", group = "MP-02")
public class PedroLocalizationPushTestOpMode extends OpMode {

    private Follower follower;
    private DcMotorEx par0Encoder;
    private DcMotorEx par1Encoder;
    private DcMotorEx perpEncoder;
    private int par0Baseline;
    private int par1Baseline;
    private int perpBaseline;
    private boolean resetWasPressed;
    private boolean emergencyStopLatched;

    @Override
    public void init() {
        RobotSafety.beginOpMode();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        par0Encoder = hardwareMap.get(DcMotorEx.class, RobotMap.ODOMETRY_PARALLEL_0);
        par1Encoder = hardwareMap.get(DcMotorEx.class, RobotMap.ODOMETRY_PARALLEL_1);
        perpEncoder = hardwareMap.get(DcMotorEx.class, RobotMap.ODOMETRY_PERPENDICULAR);
        captureEncoderBaselines();
        zeroFollower();
        RobotSafety.registerShutdown(this::zeroFollower);

        telemetry.addLine("MP-02: LOCALIZACION PEDRO, EMPUJE MANUAL");
        telemetry.addLine("No use joysticks; este modo nunca habilita movimiento.");
        telemetry.addLine("A = reset pose | BACK = E-stop");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        updateLocalizationOnly();
        publishTelemetry("INIT");
    }

    @Override
    public void loop() {
        if (gamepad1.back) {
            emergencyStopLatched = true;
            long requestNanos = System.nanoTime();
            zeroFollower();
            RobotSafety.stopAllTimed("MP02_PEDRO_PUSH_E_STOP", requestNanos);
            requestOpModeStop();
            return;
        }

        boolean resetPressed = gamepad1.a;
        if (resetPressed && !resetWasPressed) {
            zeroFollower();
            follower.setPose(new Pose(0, 0, 0));
            captureEncoderBaselines();
        }
        resetWasPressed = resetPressed;

        updateLocalizationOnly();
        publishTelemetry("RUNNING - MOTORS FORCED ZERO");
    }

    private void updateLocalizationOnly() {
        zeroFollower();
        follower.update();
        zeroFollower();
    }

    private void zeroFollower() {
        if (follower != null) {
            follower.breakFollowing();
        }
    }

    private void captureEncoderBaselines() {
        par0Baseline = par0Encoder.getCurrentPosition();
        par1Baseline = par1Encoder.getCurrentPosition();
        perpBaseline = perpEncoder.getCurrentPosition();
    }

    private void publishTelemetry(String state) {
        Pose pose = follower.getPose();
        telemetry.addData("Estado", state);
        telemetry.addData("Pedro/X in", "%.3f", pose.getX());
        telemetry.addData("Pedro/Y in", "%.3f", pose.getY());
        telemetry.addData("Pedro/Heading deg", "%.2f", Math.toDegrees(pose.getHeading()));
        int par0RawDelta = par0Encoder.getCurrentPosition() - par0Baseline;
        int par1RawDelta = par1Encoder.getCurrentPosition() - par1Baseline;
        int perpRawDelta = perpEncoder.getCurrentPosition() - perpBaseline;
        telemetry.addData("Raw delta par0/right", par0RawDelta);
        telemetry.addData("Raw delta par1/left", par1RawDelta);
        telemetry.addData("Raw delta perp", perpRawDelta);
        telemetry.addData("Pedro delta par0/right", "%.0f",
                par0RawDelta * Constants.TBD_RIGHT_ENCODER_DIRECTION);
        telemetry.addData("Pedro delta par1/left", "%.0f",
                par1RawDelta * Constants.TBD_LEFT_ENCODER_DIRECTION);
        telemetry.addData("Pedro delta perp", "%.0f",
                perpRawDelta * Constants.TBD_STRAFE_ENCODER_DIRECTION);
        telemetry.addData("Offsets calibrados", Constants.LOCALIZER_OFFSETS_CALIBRATED);
        telemetry.addData("E-stop latched", emergencyStopLatched);
        telemetry.addLine("Empuje recto, strafe y giro a mano; no use joysticks.");
        telemetry.update();
    }

    @Override
    public void stop() {
        long requestNanos = System.nanoTime();
        zeroFollower();
        if (emergencyStopLatched) {
            RobotSafety.stopAll();
        } else {
            RobotSafety.stopAllTimed("MP02_PEDRO_PUSH_STOP", requestNanos);
        }
    }
}
