package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotMap;

public class KickerSubsystem extends SubsystemBase {

    private final MotorEx kickerMotor;

    public KickerSubsystem(HardwareMap hardwareMap) {
        kickerMotor = new MotorEx(hardwareMap, RobotMap.KICKER_MOTOR);

        kickerMotor.setRunMode(Motor.RunMode.RawPower);

        // IMPORTANTE: Usar BRAKE para que el golpe sea seco y regrese rápido al soltar
        kickerMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        kickerMotor.setInverted(RobotMap.KICKER_IS_INVERTED);
    }

    /**
     * Aplica potencia para golpear.
     * La duración del golpe dependerá del comando (WaitCommand).
     */
    public void kick() {
        kickerMotor.set(LowAltitudeConstants.KICKER_OUT_SPEED);
    }

    /**
     * Retrae el kicker manualmente si se atoró.
     */
    public void reverse() {
        kickerMotor.set(LowAltitudeConstants.KICKER_REVERSE_SPEED);
    }

    /**
     * Corta la energía inmediatamente.
     */
    public void stop() {
        kickerMotor.stopMotor(); // Al ser BRAKE, se detendrá de golpe.
    }
    public Action cargar() {
        return (telemetryPacket) -> {

            kickerMotor.set(0.7);

            return false;
        };
    }
    public Action off() {
        return (telemetryPacket) -> {

            kickerMotor.set(0.0);

            return false;
        };
    }
}