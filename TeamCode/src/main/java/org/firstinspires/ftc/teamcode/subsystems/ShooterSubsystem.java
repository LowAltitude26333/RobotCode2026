package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotMap;

public class ShooterSubsystem extends SubsystemBase {

    // 1. UN SOLO MOTOR (Dejamos únicamente al líder)
    private final MotorEx motorLeader;
    private final VoltageSensor batteryVoltageSensor;
    private final Telemetry telemetry;

    private PIDController pidController;
    private SimpleMotorFeedforward feedforward;

    private double targetShooterRPM = 0;
    private boolean isBangBangEnabled = true;

    // --- Variables de Seguridad ---
    private int loopCycleCount = 0;
    private double lastValidRPM = 0;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Inicialización del Motor Único
        motorLeader = new MotorEx(hardwareMap, RobotMap.SHOOTER_MOTOR_1,
                LowAltitudeConstants.MOTOR_TICKS_PER_REV,
                LowAltitudeConstants.MOTOR_MAX_RPM);

        configureMotors();
        reloadControllers();
    }

    private void configureMotors() {
        // 1. Invertir Motor si es necesario
        motorLeader.setInverted(RobotMap.SHOOTER_UP_MOTOR_IS_INVERTED);

        // 2. Comportamiento Zero Power (Float es ideal para evitar dañar la caja de engranes)
        motorLeader.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // 3. RESETEO MANUAL SEGURO
        motorLeader.motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // 4. Configurar modo Raw Power (Sin Encoder nativo) para control manual PIDF en FTCLib
        motorLeader.motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 5. Sincronizar estado interno con FTCLib
        motorLeader.setRunMode(Motor.RunMode.RawPower);
    }

    public void reloadControllers() {
        pidController = new PIDController(
                LowAltitudeConstants.SHOOTER_KP,
                LowAltitudeConstants.SHOOTER_KI,
                LowAltitudeConstants.SHOOTER_KD
        );
        feedforward = new SimpleMotorFeedforward(
                LowAltitudeConstants.SHOOTER_KS,
                LowAltitudeConstants.SHOOTER_KV,
                LowAltitudeConstants.SHOOTER_KA
        );
    }

    public void setTargetRPM(double shooterRPM) {
        if (Math.abs(shooterRPM - targetShooterRPM) > 500) {
            pidController.reset();
        }
        this.targetShooterRPM = shooterRPM;
    }

    public void setTargetEnum(LowAltitudeConstants.TargetRPM preset) {
        setTargetRPM(preset.targetRPM);
    }

    public void stop() {
        targetShooterRPM = 0;
        motorLeader.set(0);
    }

    /**
     * Devuelve la velocidad real FILTRADA usando solo el encoder del motor líder.
     */
    public double getActualShooterRPM() {
        // SEGURIDAD 1: Warm-Up
        if (loopCycleCount < 30) {
            return 0;
        }

        double motorVelocityTicks = motorLeader.getCorrectedVelocity();
        double motorRPM = (motorVelocityTicks / LowAltitudeConstants.MOTOR_TICKS_PER_REV) * 60.0;
        double currentRPM = motorRPM * LowAltitudeConstants.SHOOTER_GEAR_RATIO;

        // SEGURIDAD 2: Filtro de Picos "Imposibles"
        if (Math.abs(currentRPM) > 20000) {
            return 0;
        }

        lastValidRPM = currentRPM;
        return currentRPM;
    }

    public boolean isReady() {
        return targetShooterRPM > 0 &&
                Math.abs(targetShooterRPM - getActualShooterRPM()) <= LowAltitudeConstants.SHOOTER_TOLERANCE_RPM;
    }

    @Override
    public void periodic() {
        loopCycleCount++;

        // 1. Obtener velocidad real actual
        double currentShooterRPM = getActualShooterRPM();

        // Si el objetivo es 0, apagamos por completo el motor y salimos
        if (targetShooterRPM == 0) {
            motorLeader.set(0);
            return;
        }

        // --- CONVERSIÓN ---
        double targetMotorRPM = targetShooterRPM / LowAltitudeConstants.SHOOTER_GEAR_RATIO;
        double currentMotorRPM = currentShooterRPM / LowAltitudeConstants.SHOOTER_GEAR_RATIO;
        double errorShooterRPM = targetShooterRPM - currentShooterRPM;

        double targetVoltage;

        // --- LÓGICA DE CONTROL (Calculada para el motor único) ---

        // A) BANG-BANG (Aceleración y Recuperación Rápida)
        if (isBangBangEnabled && errorShooterRPM > LowAltitudeConstants.SHOOTER_BANG_BANG_THRESHOLD) {
            targetVoltage = 14.0;
        }
        // B) COAST LOGIC (Frenado pasivo si nos pasamos de revoluciones)
        else if (errorShooterRPM < -50) {
            targetVoltage = 0;
        }
        // C) PIDF (Mantenimiento estable del RPM)
        else {
            double ffVolts = feedforward.calculate(targetMotorRPM);
            double pidVolts = pidController.calculate(currentMotorRPM, targetMotorRPM);
            targetVoltage = ffVolts + pidVolts;
        }

        // --- CORRECCIÓN POR VOLTAJE DE BATERÍA ---
        double batteryVoltage = batteryVoltageSensor.getVoltage();
        if (batteryVoltage <= 1.0) {
            batteryVoltage = 12.5;
        }

        double finalPower = targetVoltage / batteryVoltage;

        // CLAMP DE SEGURIDAD (0.0 a 1.0)
        finalPower = Math.max(0.0, Math.min(1.0, finalPower));

        // Aplicar la potencia calculada al único motor activo
        motorLeader.set(finalPower);

        // Telemetría de diagnóstico
        telemetry.addData("Shooter/Target", targetShooterRPM);
        telemetry.addData("Shooter/Actual", currentShooterRPM);
        telemetry.addData("Shooter/Power", finalPower);
        telemetry.addData("Shooter/LoopCount", loopCycleCount);
    }
}