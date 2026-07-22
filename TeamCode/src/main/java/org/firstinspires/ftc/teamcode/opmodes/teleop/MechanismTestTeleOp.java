package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.opmodes.SafeCommandOpMode;
import org.firstinspires.ftc.teamcode.safety.TurretArmingStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/** Integrated mechanism test without drive, camera, hood, or kicker CRServo output. */
@TeleOp(name = "PRUEBA: MECANISMOS SIN SERVO", group = "Test")
public final class MechanismTestTeleOp extends SafeCommandOpMode {
    private static final double SHOOTER_CONSTANT_POWER = 0.90;
    private static final long TURRET_PRESET_TIMEOUT_MS = 5000;

    // Four deliberately conservative test positions inside the active -983/+1070 limits.
    private static final int TURRET_DPAD_LEFT_TICKS = -150;
    private static final int TURRET_DPAD_UP_TICKS = -50;
    private static final int TURRET_DPAD_DOWN_TICKS = 50;
    private static final int TURRET_DPAD_RIGHT_TICKS = 150;

    private ShooterSubsystem shooter;
    private KickerSubsystem kicker;
    private IntakeSubsystem intake;
    private TurretSubsystem turret;
    private TurretArmingStateMachine turretArming;

    private boolean intakeOn;
    private boolean kickerOn;
    private boolean shooterOn;
    private String intakeResult = "OFF";
    private String kickerResult = "OFF";
    private String shooterResult = "OFF";

    private boolean previousG1A;
    private boolean previousG1B;
    private boolean previousG1X;
    private boolean previousG1Y;
    private boolean previousG2A;
    private boolean previousG2B;
    private boolean previousDpadLeft;
    private boolean previousDpadUp;
    private boolean previousDpadDown;
    private boolean previousDpadRight;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        if (!shooter.setCommissioningOutputPowerLimit(SHOOTER_CONSTANT_POWER)
                || !shooter.enableCommissioningPhysicalOutput()) {
            shooterResult = "BLOCKED_OUTPUT_SETUP";
        }
        // Owner-level interlock: this OpMode can never command nonzero CRServo power.
        kicker = new KickerSubsystem(hardwareMap, false);
        intake = new IntakeSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        turretArming = new TurretArmingStateMachine(
                LowAltitudeConstants.TurretConstants.TURRET_ARM_HOLD_MS);

        stopPersistentMechanisms();
        telemetry.addLine("PRUEBA DE MECANISMOS SIN DRIVE Y SIN KICKER SERVO");
        telemetry.addLine("Centre torreta; G2 START+BACK 1 s durante INIT confirma cero.");
        telemetry.addLine("G1 A/B intake | X/Y kicker | DPAD presets");
        telemetry.addLine("G2 A/B shooter constante 0.90 | G1 BACK E-STOP");
        telemetry.update();
    }

    @Override
    protected void duringInitLoop() {
        long nowNanos = System.nanoTime();
        if (turretArming.update(gamepad2.start && gamepad2.back, nowNanos)) {
            turret.confirmCenteredAndResetEncoder();
        }

        telemetry.addLine("=== ARMADO DE TORRETA ===");
        telemetry.addData("Turret/Arming", turretArming.getState());
        telemetry.addData("Turret/Arm progress", "%.0f%%",
                turretArming.getProgress(nowNanos) * 100.0);
        telemetry.addData("Turret/Zero", turret.getZeroState());
        telemetry.addData("Turret/Ticks", turret.getPosition());
        telemetry.addData("Kicker/Servo allowed", kicker.isServoAllowedByOwner());
        telemetry.addData("Kicker/Servo active", kicker.isServoActive());
        telemetry.addLine("Alinee la marca antes de confirmar; INIT no mueve mecanismos.");
    }

    @Override
    public void run() {
        shooter.reloadControllersIfConstantsChanged();

        updateIntakeButtons();
        updateKickerButtons();
        updateShooterButtons();
        updateTurretPresetButtons();
        applyPersistentMechanismRequests();

        // Runs subsystem health checks and turret preset completion/timeout logic.
        super.run();

        if (shooter.isFaultLatched()) {
            shooterOn = false;
            shooterResult = "STOPPED_FAULT";
        }

        publishMechanismTelemetry();
        telemetry.update();
    }

    private void updateIntakeButtons() {
        if (gamepad1.b && !previousG1B) {
            intakeOn = false;
            intakeResult = "OFF";
        } else if (gamepad1.a && !previousG1A) {
            intakeOn = true;
            intakeResult = "ON";
        }
        previousG1A = gamepad1.a;
        previousG1B = gamepad1.b;
    }

    private void updateKickerButtons() {
        if (gamepad1.y && !previousG1Y) {
            kickerOn = false;
            kickerResult = "OFF";
        } else if (gamepad1.x && !previousG1X) {
            kickerOn = true;
            kickerResult = "MOTOR_ON";
        }
        previousG1X = gamepad1.x;
        previousG1Y = gamepad1.y;
    }

    private void updateShooterButtons() {
        if (gamepad2.b && !previousG2B) {
            shooterOn = false;
            shooter.stop();
            shooterResult = "OFF";
        } else if (gamepad2.a && !previousG2A) {
            shooterOn = true;
            shooterResult = "CONSTANT_POWER_REQUESTED";
        }
        previousG2A = gamepad2.a;
        previousG2B = gamepad2.b;
    }

    private void updateTurretPresetButtons() {
        if (gamepad1.dpad_left && !previousDpadLeft) {
            turret.requestPresetPosition(TURRET_DPAD_LEFT_TICKS, TURRET_PRESET_TIMEOUT_MS);
        } else if (gamepad1.dpad_up && !previousDpadUp) {
            turret.requestPresetPosition(TURRET_DPAD_UP_TICKS, TURRET_PRESET_TIMEOUT_MS);
        } else if (gamepad1.dpad_down && !previousDpadDown) {
            turret.requestPresetPosition(TURRET_DPAD_DOWN_TICKS, TURRET_PRESET_TIMEOUT_MS);
        } else if (gamepad1.dpad_right && !previousDpadRight) {
            turret.requestPresetPosition(TURRET_DPAD_RIGHT_TICKS, TURRET_PRESET_TIMEOUT_MS);
        }
        previousDpadLeft = gamepad1.dpad_left;
        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;
        previousDpadRight = gamepad1.dpad_right;
    }

    private void applyPersistentMechanismRequests() {
        if (intakeOn) {
            intake.intakeOn();
        } else {
            intake.intakeOff();
        }

        if (kickerOn) {
            kicker.kick();
        } else {
            kicker.stop();
        }

        if (shooterOn) {
            if (shooter.runCommissioningAtConstantPower(SHOOTER_CONSTANT_POWER)) {
                shooterResult = "CONSTANT_POWER_ON";
            } else {
                shooterOn = false;
                shooterResult = "BLOCKED_FAULT_OR_OUTPUT";
            }
        } else {
            shooter.stop();
        }
    }

    private void stopPersistentMechanisms() {
        intakeOn = false;
        kickerOn = false;
        shooterOn = false;
        intake.intakeOff();
        kicker.stop();
        shooter.stop();
        turret.stop();
    }

    private void publishMechanismTelemetry() {
        telemetry.addLine("=== PRUEBA MECANISMOS SIN SERVO ===");
        telemetry.addData("Intake", intakeResult);
        telemetry.addData("Kicker/Motor", kickerResult);
        telemetry.addData("Kicker/Servo allowed", kicker.isServoAllowedByOwner());
        telemetry.addData("Kicker/Servo active", kicker.isServoActive());

        telemetry.addData("Shooter/Requested", shooterOn);
        telemetry.addData("Shooter/Result", shooterResult);
        telemetry.addData("Shooter/Constant power", "%.2f",
                shooter.getCommissioningConstantPower());
        telemetry.addData("Shooter/Applied power", "%.2f", shooter.getLastAppliedPower());
        telemetry.addData("Shooter/Actual RPM", "%.1f", shooter.getActualShooterRPM());
        telemetry.addData("Shooter/Health", shooter.getHealthState());
        telemetry.addData("Shooter/Fault", shooter.getFaultReason());
        telemetry.addData("Shooter/Battery", "%.2f V", shooter.getBatteryVoltage());

        telemetry.addData("Turret/Armed", turret.isArmed());
        telemetry.addData("Turret/Zero", turret.getZeroState());
        telemetry.addData("Turret/Ticks", turret.getPosition());
        telemetry.addData("Turret/Preset target", turret.getPresetTargetTicks());
        telemetry.addData("Turret/Preset active", turret.isPresetMoveActive());
        telemetry.addData("Turret/Power", turret.getLastAppliedPower());
        telemetry.addData("Turret/Result", turret.getLastCommissioningResult());

        telemetry.addLine("G1 A=INTAKE ON | B=INTAKE OFF | X=KICKER ON | Y=KICKER OFF");
        telemetry.addLine("G1 DPAD LEFT=-150 | UP=-50 | DOWN=+50 | RIGHT=+150 ticks");
        telemetry.addLine("G2 A=SHOOTER ON 0.90 CONSTANTE | B=SHOOTER OFF");
        telemetry.addLine("G1 BACK=E-STOP | Driver Station STOP siempre apaga todo");
    }
}
