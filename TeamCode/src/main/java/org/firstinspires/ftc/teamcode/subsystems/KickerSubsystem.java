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

public class KickerSubsystem extends SubsystemBase {

    private final MotorEx kickerMotor;
    private final CRServo kickerServo;
    private final boolean kickerServoEnabledAtInit;
    private final boolean kickerServoAllowedByOwner;

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
     * Aplica potencia para golpear.
     * La duración del golpe dependerá del comando (WaitCommand).
     */
    public void kick() {
        kickerMotor.set(LowAltitudeConstants.KICKER_OUT_SPEED);
        if (isServoActive()) {
            kickerServo.setPower(LowAltitudeConstants.KICKER_SERVO_FORWARD_POWER);
        }
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
     * Corta la energía inmediatamente.
     */
    public void stop() {
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
