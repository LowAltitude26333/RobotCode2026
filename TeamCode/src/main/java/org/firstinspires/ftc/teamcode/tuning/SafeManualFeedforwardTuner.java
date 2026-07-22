package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Profiles;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeProfile;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.EncoderGroup;
import com.acmerobotics.roadrunner.ftc.EncoderRef;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RollingThreeMedian;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * Road Runner 0.1.23 ManualFeedforwardTuner with an explicit motor-zero path.
 *
 * <p>The tuning behavior and controls match the upstream tuner. The finally block
 * is intentional: it guarantees zero drive power on Stop or on any exception.</p>
 */
public final class SafeManualFeedforwardTuner extends LinearOpMode {
    public static double DISTANCE = 64.0;

    private static final PoseVelocity2d ZERO_POWER =
            new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);

    private enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    private final DriveViewFactory driveViewFactory;

    public SafeManualFeedforwardTuner(DriveViewFactory driveViewFactory) {
        this.driveViewFactory = driveViewFactory;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry());

        DriveView view = driveViewFactory.make(hardwareMap);
        TimeProfile profile = new TimeProfile(Profiles.constantProfile(
                DISTANCE,
                0.0,
                view.getMaxVel(),
                view.getMinAccel(),
                view.getMaxAccel()).baseProfile);

        Mode mode = Mode.TUNING_MODE;
        telemetry.addLine("Ready! Mantenga acceso a STOP. Y=manual, B=reanudar tuner.");
        telemetry.update();
        telemetry.clearAll();

        // Establish the safe state before the OpMode is enabled.
        view.setDrivePowers(ZERO_POWER);

        try {
            waitForStart();
            if (isStopRequested()) {
                return;
            }

            boolean movingForwards = true;
            double startTs = System.nanoTime() / 1e9;

            List<Integer> lastPositions = new ArrayList<>();
            List<ElapsedTime> lastTimes = new ArrayList<>();
            List<RollingThreeMedian> velocityEstimates = new ArrayList<>();
            for (int i = 0; i < view.getForwardEncs().size(); i++) {
                lastPositions.add(0);
                lastTimes.add(new ElapsedTime());
                velocityEstimates.add(new RollingThreeMedian());
            }

            while (opModeIsActive()) {
                telemetry.addData("mode", mode);

                if (mode == Mode.TUNING_MODE) {
                    if (gamepad1.y) {
                        mode = Mode.DRIVER_MODE;
                        view.setDrivePowers(ZERO_POWER);
                        telemetry.update();
                        continue;
                    }

                    for (EncoderGroup group : view.getEncoderGroups()) {
                        group.bulkRead();
                    }

                    List<EncoderRef> forwardEncoders = view.getForwardEncs();
                    for (int i = 0; i < forwardEncoders.size(); i++) {
                        PositionVelocityPair positionVelocity =
                                view.encoder(forwardEncoders.get(i)).getPositionAndVelocity();
                        double velocity;
                        if (positionVelocity.velocity == null) {
                            int lastPosition = lastPositions.get(i);
                            lastPositions.set(i, positionVelocity.position);
                            velocity = velocityEstimates.get(i).update(
                                    (positionVelocity.position - lastPosition)
                                            / lastTimes.get(i).seconds());
                            lastTimes.get(i).reset();
                        } else {
                            velocity = positionVelocity.velocity;
                        }
                        telemetry.addData("v" + i, view.getInPerTick() * velocity);
                    }

                    double timestamp = System.nanoTime() / 1e9;
                    double profileTime = timestamp - startTs;
                    if (profileTime > profile.duration) {
                        movingForwards = !movingForwards;
                        startTs = timestamp;
                        profileTime = 0.0;
                    }

                    DualNum<Time> target = profile.get(profileTime).drop(1);
                    if (!movingForwards) {
                        target = target.unaryMinus();
                    }

                    telemetry.addData("vref", target.get(0));
                    double voltage = view.getVoltageSensor().getVoltage();
                    if (!Double.isFinite(voltage) || voltage <= 0.0) {
                        view.setDrivePowers(ZERO_POWER);
                        telemetry.addLine("ERROR: voltaje invalido; motores en cero.");
                        requestOpModeStop();
                        continue;
                    }

                    double power = view.getFeedforwardFactory().make().compute(target) / voltage;
                    view.setDrivePowers(new PoseVelocity2d(new Vector2d(power, 0.0), 0.0));
                } else {
                    if (gamepad1.b) {
                        mode = Mode.TUNING_MODE;
                        movingForwards = true;
                        startTs = System.nanoTime() / 1e9;
                        view.setDrivePowers(ZERO_POWER);
                        telemetry.update();
                        continue;
                    }

                    view.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x),
                            -gamepad1.right_stick_x));
                }

                telemetry.update();
            }
        } finally {
            // Required even when Stop interrupts the OpMode or an exception occurs.
            view.setDrivePowers(ZERO_POWER);
        }
    }
}
