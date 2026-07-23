package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.shooter.ShooterFeedbackSign;

import java.util.ArrayList;
import java.util.List;

/**
 * Lowest-level shooter encoder diagnostic.
 *
 * <p>No FTCLib, scheduler, PID, feedforward, encoder reset, or nonzero motor command
 * is used. REV bulk caching is disabled so every SDK read reaches the hub.</p>
 */
@TeleOp(name = "DIAG: Shooter Encoder RAW", group = "Diagnostic")
public final class ShooterEncoderRawDiagnosticOpMode extends OpMode {
    private DcMotorEx shooterMotor;
    private List<LynxModule> lynxModules;
    private final List<LynxModule.BulkCachingMode> originalCachingModes =
            new ArrayList<>();

    private int startRawTicks;
    private int previousRawTicks;
    private int minimumRawTicks;
    private int maximumRawTicks;
    private long changedSampleCount;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry());

        lynxModules = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : lynxModules) {
            originalCachingModes.add(module.getBulkCachingMode());
            module.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }

        shooterMotor = hardwareMap.get(DcMotorEx.class, RobotMap.SHOOTER_MOTOR);
        shooterMotor.setPower(0.0);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int rawTicks = shooterMotor.getCurrentPosition();
        startRawTicks = rawTicks;
        previousRawTicks = rawTicks;
        minimumRawTicks = rawTicks;
        maximumRawTicks = rawTicks;

        telemetry.addLine("Encoder RAW listo; potencia fijada a 0.");
        telemetry.addLine("Presione PLAY y después gire el eje del motor.");
        publish(rawTicks);
        telemetry.update();
    }

    @Override
    public void start() {
        int rawTicks = shooterMotor.getCurrentPosition();
        startRawTicks = rawTicks;
        previousRawTicks = rawTicks;
        minimumRawTicks = rawTicks;
        maximumRawTicks = rawTicks;
        changedSampleCount = 0;
    }

    @Override
    public void loop() {
        shooterMotor.setPower(0.0);

        int rawTicks = shooterMotor.getCurrentPosition();
        if (rawTicks != previousRawTicks) {
            changedSampleCount++;
            previousRawTicks = rawTicks;
        }
        minimumRawTicks = Math.min(minimumRawTicks, rawTicks);
        maximumRawTicks = Math.max(maximumRawTicks, rawTicks);

        publish(rawTicks);
        telemetry.update();
    }

    private void publish(int rawTicks) {
        int outwardTicks = ShooterFeedbackSign.outwardPositive(
                rawTicks,
                RobotMap.SHOOTER_ENCODER_IS_INVERTED);
        int outwardStartTicks = ShooterFeedbackSign.outwardPositive(
                startRawTicks,
                RobotMap.SHOOTER_ENCODER_IS_INVERTED);
        double rawTicksPerSecond = shooterMotor.getVelocity();
        double outwardTicksPerSecond = ShooterFeedbackSign.outwardPositive(
                rawTicksPerSecond,
                RobotMap.SHOOTER_ENCODER_IS_INVERTED);
        double outwardRpm = outwardTicksPerSecond
                * 60.0 / LowAltitudeConstants.MOTOR_TICKS_PER_REV;

        telemetry.addData("Diag/Hardware name", RobotMap.SHOOTER_MOTOR);
        telemetry.addData("Diag/Connection", shooterMotor.getConnectionInfo());
        telemetry.addData("Diag/SDK mode", shooterMotor.getMode());
        telemetry.addData("Diag/Bulk caching", "OFF (%d hubs)", lynxModules.size());
        telemetry.addData("Diag/Commanded power", 0.0);
        telemetry.addData("Diag/Raw ticks", rawTicks);
        telemetry.addData("Diag/Raw delta", rawTicks - startRawTicks);
        telemetry.addData("Diag/Outward ticks", outwardTicks);
        telemetry.addData("Diag/Outward delta", outwardTicks - outwardStartTicks);
        telemetry.addData("Diag/Raw ticks per sec", "%.3f", rawTicksPerSecond);
        telemetry.addData("Diag/Outward ticks per sec", "%.3f", outwardTicksPerSecond);
        telemetry.addData("Diag/Outward RPM", "%.3f", outwardRpm);
        telemetry.addData("Diag/Changed samples", changedSampleCount);
        telemetry.addData("Diag/Raw min..max", "%d .. %d",
                minimumRawTicks, maximumRawTicks);
        telemetry.addLine("Esperado: 1 vuelta=28 ticks; 10 vueltas=280 ticks.");
    }

    @Override
    public void stop() {
        if (shooterMotor != null) {
            shooterMotor.setPower(0.0);
        }
        for (int i = 0; i < originalCachingModes.size(); i++) {
            LynxModule module = lynxModules.get(i);
            module.setBulkCachingMode(originalCachingModes.get(i));
            module.clearBulkCache();
        }
    }
}
