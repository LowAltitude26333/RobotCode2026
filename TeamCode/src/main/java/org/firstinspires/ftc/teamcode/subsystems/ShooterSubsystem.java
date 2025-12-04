package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotMap;

public class ShooterSubsystem extends SubsystemBase {

    private final MotorEx motorLeader;
    private final MotorEx motorFollower;
    private final PIDFController pidfController;
    private final Telemetry telemetry;

    private double targetRPM = 0;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        motorLeader = new MotorEx(hardwareMap, RobotMap.SHOOTER_MOTOR_1);
        motorFollower = new MotorEx(hardwareMap, RobotMap.SHOOTER_MOTOR_2);

        motorLeader.setInverted(RobotMap.SHOOTER_UP_MOTOR_IS_INVERTED);
        motorFollower.setInverted(RobotMap.SHOOTER_DOWN_MOTOR_IS_INVERTED);

        motorLeader.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motorFollower.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        motorLeader.setRunMode(Motor.RunMode.RawPower);
        motorFollower.setRunMode(Motor.RunMode.RawPower);

        motorLeader.resetEncoder();

        pidfController = new PIDFController(
                LowAltitudeConstants.SHOOTER_KP,
                LowAltitudeConstants.SHOOTER_KI,
                LowAltitudeConstants.SHOOTER_KD,
                LowAltitudeConstants.SHOOTER_KF
        );
    }


    /*
     * ==================================================================
     * Comandos TeleOp
     * ==================================================================
     */
    // Solo guarda el valor, no mueve nada aún
    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
        // Si cambiamos de objetivo, reseteamos el acumulado del PID para evitar saltos
        pidfController.reset();
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getShooterRPM() {
        double motorTicksPerSec = motorLeader.getVelocity();
        double motorRPM = (motorTicksPerSec * 60.0) / LowAltitudeConstants.TICKS_PER_REV;
        return motorRPM * LowAltitudeConstants.SHOOTER_VELOCITY_MULTIPLIER;
    }

    public void runPID() {
        if (targetRPM == 0) {
            stop();
            return;
        }

        double currentRPM = getShooterRPM();
        double error = targetRPM - currentRPM;

        double finalPower;

        // --- LÓGICA HÍBRIDA MEJORADA ---

        // CASO DE EMERGENCIA (Caída fuerte por disparo)
        // Aquí sí le damos permiso de usar el 100% (1.0) para levantar rápido
        if (error > LowAltitudeConstants.SHOOTER_RECOVERY_THRESHOLD) {
            finalPower = LowAltitudeConstants.SHOOTER_BOOST_SPEED;
        }
        // CASO DE MANTENIMIENTO (Ya estamos cerca o estables)
        // Aquí respetamos tu límite de seguridad de 0.9
        else {
            double pidOutput = pidfController.calculate(currentRPM, targetRPM);
            finalPower = Math.max(-LowAltitudeConstants.SHOOTER_MAX_SPEED,
                    Math.min(pidOutput, LowAltitudeConstants.SHOOTER_MAX_SPEED));
        }

        motorLeader.set(finalPower);
        motorFollower.set(finalPower);
    }

    public void driveShooter(double power){
        motorLeader.set(power);
        motorFollower.set(power);
    }

    // Método auxiliar para usar el Enum directamente
    public void setTargetEnum(LowAltitudeConstants.TargetRPM preset) {
        setTargetRPM(preset.targetRPM);
    }

    // Apagado de emergencia o descanso
    public void stop() {
        targetRPM = 0;
        motorLeader.set(0);
        motorFollower.set(0);
        motorLeader.stopMotor();
        motorFollower.stopMotor();
    }

    public boolean onTarget() {
        return Math.abs(targetRPM - getShooterRPM()) < LowAltitudeConstants.RPM_OFFSET;
    }

    @Override
    public void periodic() {
        telemetry.addData("SHOOTER Target", targetRPM);
        telemetry.addData("SHOOTER Actual", getShooterRPM());
        telemetry.addData("SHOOTER Velocity", motorLeader.getVelocity());
        telemetry.addData("SHOOTER Power", motorLeader.get());
    }

    /*
    =========================================================================
    Comandos Autónomo
    =========================================================================
     */

    public Action setRPMAutonomous(double RPM) {
        return(telemetryPacket) -> {
            setTargetRPM(RPM);
            return false;
        };
    }

    public Action runPIDAutonomous() {
        return (telemetryPacket) -> {
            runPID();
            return false;
        };
    }


    public Action waitUntilTargetRPMAutonomous() {
        return (telemetryPacket) -> onTarget();
        // onTarget es true ==> false
    }

    public Action stopAutonomous() {
        return (telemetryPacket) -> {
            stop();
            return false;
        };
    }
}