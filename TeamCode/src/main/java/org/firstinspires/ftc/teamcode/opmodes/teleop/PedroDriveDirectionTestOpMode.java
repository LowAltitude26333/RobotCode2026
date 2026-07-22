package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSafety;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/** Low-power, hold-to-run drivetrain direction gate before Pedro autonomous motion. */
@TeleOp(name = "MP02 Pedro Drive Direction Test", group = "MP-02")
public class PedroDriveDirectionTestOpMode extends OpMode {
    private static final double TEST_POWER = 0.50;

    private Follower follower;
    private boolean driveStarted;
    private boolean emergencyStopLatched;
    private String commandState = "RELEASED_ZERO";

    @Override
    public void init() {
        RobotSafety.beginOpMode();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        shutdownDrive();
        RobotSafety.registerShutdown(this::shutdownDrive);
        publishTelemetry();
    }

    @Override
    public void init_loop() {
        shutdownDrive();
        commandState = "INIT_ZERO";
        publishTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        driveStarted = true;
        commandZero();
    }

    @Override
    public void loop() {
        if (gamepad1.back) {
            emergencyStopLatched = true;
            long requestNanos = System.nanoTime();
            shutdownDrive();
            RobotSafety.stopAllTimed("MP02_PEDRO_DRIVE_E_STOP", requestNanos);
            requestOpModeStop();
            return;
        }

        int pressedCount = countPressedDirections();
        double forward = 0;
        double strafe = 0;
        double turn = 0;

        if (pressedCount > 1) {
            commandState = "REJECTED_MULTIPLE_INPUTS";
        } else if (gamepad1.a) {
            forward = TEST_POWER;
            commandState = "FORWARD";
        } else if (gamepad1.b) {
            forward = -TEST_POWER;
            commandState = "BACKWARD";
        } else if (gamepad1.x) {
            strafe = TEST_POWER;
            commandState = "STRAFE_LEFT";
        } else if (gamepad1.y) {
            strafe = -TEST_POWER;
            commandState = "STRAFE_RIGHT";
        } else if (gamepad1.dpad_left) {
            turn = TEST_POWER;
            commandState = "ROTATE_CCW";
        } else if (gamepad1.dpad_right) {
            turn = -TEST_POWER;
            commandState = "ROTATE_CW";
        } else {
            commandState = "RELEASED_ZERO";
        }

        follower.setTeleOpDrive(forward, strafe, turn, true);
        follower.update();
        publishTelemetry();
    }

    private int countPressedDirections() {
        int count = 0;
        if (gamepad1.a) count++;
        if (gamepad1.b) count++;
        if (gamepad1.x) count++;
        if (gamepad1.y) count++;
        if (gamepad1.dpad_left) count++;
        if (gamepad1.dpad_right) count++;
        return count;
    }

    private void commandZero() {
        if (follower != null) {
            if (driveStarted) {
                follower.setTeleOpDrive(0, 0, 0, true);
                follower.update();
            }
        }
    }

    private void shutdownDrive() {
        if (follower != null) {
            commandZero();
            follower.breakFollowing();
        }
    }

    private void publishTelemetry() {
        Pose pose = follower.getPose();
        telemetry.addLine("MP-02 PEDRO: DIRECCION A POTENCIA 0.50");
        telemetry.addLine("Mantenga UN boton; soltar = cero.");
        telemetry.addData("Comando", commandState);
        telemetry.addData("Pedro manual activo", follower.isTeleopDrive());
        telemetry.addData("X in", "%.3f", pose.getX());
        telemetry.addData("Y in", "%.3f", pose.getY());
        telemetry.addData("Heading deg", "%.2f", Math.toDegrees(pose.getHeading()));
        telemetry.addData("E-stop latched", emergencyStopLatched);
        telemetry.addLine("A=forward B=back X=left Y=right");
        telemetry.addLine("D-pad left=CCW right=CW | BACK=E-stop");
        telemetry.update();
    }

    @Override
    public void stop() {
        long requestNanos = System.nanoTime();
        shutdownDrive();
        if (emergencyStopLatched) {
            RobotSafety.stopAll();
        } else {
            RobotSafety.stopAllTimed("MP02_PEDRO_DRIVE_STOP", requestNanos);
        }
    }
}
