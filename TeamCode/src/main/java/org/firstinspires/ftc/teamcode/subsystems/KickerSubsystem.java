package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.RobotSafety;
import org.firstinspires.ftc.teamcode.safety.FeederPulseStateMachine;

public class KickerSubsystem extends SubsystemBase {

    private final MotorEx kickerMotor;
    private final CRServo kickerServo;
    private final boolean kickerServoEnabledAtInit;
    private final boolean kickerServoAllowedByOwner;
    private final FeederPulseStateMachine pulseState = new FeederPulseStateMachine(
            LowAltitudeConstants.KICKER_MAX_PULSE_MS, LowAltitudeConstants.KICKER_COOLDOWN_MS);

    public KickerSubsystem(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }

    /**
     * Creates a kicker owner that can permanently forbid servo output for its lifetime.
     * Passing false keeps the motor available while every servo path remains at zero,
     * even if the Dashboard flag was enabled before INIT.
     */
    public KickerSubsystem(HardwareMap hardwareMap, boolean allowServoForThisOwner) {
        kickerMotor = new MotorEx(hardwareMap, RobotMap.KICKER_MOTOR);
        kickerServo = hardwareMap.tryGet(CRServo.class, RobotMap.KICKER_SERVO);
        kickerServoEnabledAtInit = LowAltitudeConstants.KICKER_SERVO_ENABLED;
        kickerServoAllowedByOwner = allowServoForThisOwner;

        kickerMotor.setRunMode(Motor.RunMode.RawPower);

        // IMPORTANTE: Usar BRAKE para que el golpe sea seco y regrese rápido al soltar
        kickerMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        kickerMotor.setInverted(RobotMap.KICKER_MOTOR_IS_INVERTED);
        if (kickerServo != null) {
            kickerServo.setDirection(RobotMap.KICKER_SERVO_IS_INVERTED
                    ? DcMotorSimple.Direction.REVERSE
                    : DcMotorSimple.Direction.FORWARD);
        } else if (kickerServoEnabledAtInit && kickerServoAllowedByOwner) {
            RobotLog.addGlobalWarningMessage(
                    RobotMap.KICKER_SERVO
                            + " enabled but unavailable; kickerMotor-only mode is active");
        }
        stop();
        RobotSafety.registerShutdown(this::stop);
    }

    /**
     * Aplica potencia para golpear, acotada por {@link FeederPulseStateMachine} (FND-018):
     * se corta sola al llegar a {@code KICKER_MAX_PULSE_MS} aunque el caller siga pidiendo
     * el pulso (p.ej. un binding hold-to-run), y no vuelve a energizar hasta completar
     * {@code KICKER_COOLDOWN_MS}.
     */
    public void kick() {
        boolean pulseAllowed = pulseState.requestPulse(System.nanoTime());
        if (!pulseAllowed) {
            stopOutputsOnly();
            return;
        }
        kickerMotor.set(LowAltitudeConstants.KICKER_OUT_SPEED);
        if (isServoActive()) {
            kickerServo.setPower(LowAltitudeConstants.KICKER_SERVO_FORWARD_POWER);
        }
    }

    public FeederPulseStateMachine.State getPulseState() {
        return pulseState.getState();
    }

    /**
     * Retrae el kicker manualmente si se atoró.
     */
    public void reverse() {
        kickerMotor.set(LowAltitudeConstants.KICKER_REVERSE_SPEED);
        if (isServoActive()) {
            kickerServo.setPower(LowAltitudeConstants.KICKER_SERVO_REVERSE_POWER);
        }
    }

    /**
     * Corta la energía inmediatamente y libera el pulso (entra a cooldown si venía de
     * PULSING, por FND-018).
     */
    public void stop() {
        pulseState.release(System.nanoTime());
        stopOutputsOnly();
    }

    private void stopOutputsOnly() {
        kickerMotor.stopMotor(); // Al ser BRAKE, se detendrá de golpe.
        // Zero an available servo even when disabled, so a previous OpMode cannot leave it moving.
        if (kickerServo != null) {
            kickerServo.setPower(LowAltitudeConstants.KICKER_SERVO_STOP_POWER);
        }
    }

    public boolean isServoEnabledAtInit() {
        return kickerServoEnabledAtInit;
    }

    public boolean isServoAvailable() {
        return kickerServo != null;
    }

    public boolean isServoAllowedByOwner() {
        return kickerServoAllowedByOwner;
    }

    public boolean isServoActive() {
        return kickerServoEnabledAtInit && kickerServoAllowedByOwner && kickerServo != null;
    }

    @Override
    public void periodic() {
        // Enforce the hard cutoff even when a caller only invoked kick() once, then keep
        // COOLDOWN -> IDLE fresh for telemetry while no request is active.
        pulseState.update(System.nanoTime());
        if (pulseState.getState() != FeederPulseStateMachine.State.PULSING) {
            stopOutputsOnly();
        }
    }

    public Action cargar() {
        return (telemetryPacket) -> {

            kick();

            return false;
        };
    }
    public Action off() {
        return (telemetryPacket) -> {

            stop();

            return false;
        };
    }
}
