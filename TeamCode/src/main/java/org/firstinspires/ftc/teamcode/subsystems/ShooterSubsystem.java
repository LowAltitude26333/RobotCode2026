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

    private final MotorEx motorLeader;
    private final MotorEx motorFollower;
    private final VoltageSensor batteryVoltageSensor;
    private final Telemetry telemetry;

    private PIDController pidController;
    private SimpleMotorFeedforward feedforward;

    private double targetShooterRPM = 0;
    private boolean isBangBangEnabled = true;

    // --- NUEVO: Variables de Seguridad ---
    private int loopCycleCount = 0; // Contador de ciclos para "Warm-Up"
    private double lastValidRPM = 0; // Para recordar la última velocidad real si hay un pico

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // 1. Inicialización de Hardware
        motorLeader = new MotorEx(hardwareMap, RobotMap.SHOOTER_MOTOR_1,
                LowAltitudeConstants.MOTOR_TICKS_PER_REV,
                LowAltitudeConstants.MOTOR_MAX_RPM);

        motorFollower = new MotorEx(hardwareMap, RobotMap.SHOOTER_MOTOR_2,
                LowAltitudeConstants.MOTOR_TICKS_PER_REV,
                LowAltitudeConstants.MOTOR_MAX_RPM);

        configureMotors();
        reloadControllers();
    }

    private void configureMotors() {
        // 1. Invertir Motores
        motorLeader.setInverted(RobotMap.SHOOTER_UP_MOTOR_IS_INVERTED);
        motorFollower.setInverted(RobotMap.SHOOTER_DOWN_MOTOR_IS_INVERTED);

        // 2. Comportamiento Zero Power (Float es mejor para shooters)
        motorLeader.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motorFollower.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        // 3. RESETEO MANUAL SEGURO (CORREGIDO)
        // Accedemos al motor nativo (.motor) para asegurar que el SDK reciba las órdenes
        motorLeader.motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFollower.motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // --- LA LÍNEA MÁGICA QUE FALTABA ---
        // Tenemos que explícitamente sacarlo de Reset y ponerlo en modo "Sin Encoder" (Raw Power)
        // Usamos el objeto nativo para que FTCLib no se confunda.
        motorLeader.motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFollower.motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 4. Sincronizar FTCLib
        // Ahora le decimos a FTCLib: "Oye, estamos en RawPower, actualiza tu estado interno"
        motorLeader.setRunMode(Motor.RunMode.RawPower);
        motorFollower.setRunMode(Motor.RunMode.RawPower);
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
        motorFollower.set(0);
    }

    /**
     * Devuelve la velocidad real FILTRADA.
     * Si el sensor enloquece, devuelve 0 o el último valor conocido.
     */
    public double getActualShooterRPM() {
        // SEGURIDAD 1: Warm-Up
        // Si el robot acaba de arrancar (primeros 30 ciclos), devolvemos 0 para dejar que se estabilice.
        if (loopCycleCount < 30) {
            return 0;
        }

        double motorVelocityTicks = motorLeader.getCorrectedVelocity();
        double motorRPM = (motorVelocityTicks / LowAltitudeConstants.MOTOR_TICKS_PER_REV) * 60.0;
        double currentRPM = motorRPM * LowAltitudeConstants.SHOOTER_GEAR_RATIO;

        // SEGURIDAD 2: Filtro de Picos "Imposibles"
        // Ningún motor GoBilda llega a 20,000 RPM. Si leemos eso, es un glitch.
        if (Math.abs(currentRPM) > 20000) {
            // Opción A: Retornar 0 (Seguro para que el PID no reste potencia loca)
            return 0;
            // Opción B (Más suave): Retornar el último valor válido
            // return lastValidRPM;
        }

        lastValidRPM = currentRPM;
        return currentRPM;
    }

    public boolean isReady() {
        // Usamos el getter filtrado
        return targetShooterRPM > 0 &&
                Math.abs(targetShooterRPM - getActualShooterRPM()) <= LowAltitudeConstants.SHOOTER_TOLERANCE_RPM;
    }

    @Override
    public void periodic() {
        loopCycleCount++; // Contamos ciclos de vida

        // 1. OBTENER DATOS (Ya filtrados por el método getActualShooterRPM)
        double currentShooterRPM = getActualShooterRPM();

        if (targetShooterRPM == 0) {
            motorLeader.set(0);
            motorFollower.set(0);
            return;
        }

        // --- CONVERSIÓN ---
        double targetMotorRPM = targetShooterRPM / LowAltitudeConstants.SHOOTER_GEAR_RATIO;
        double currentMotorRPM = currentShooterRPM / LowAltitudeConstants.SHOOTER_GEAR_RATIO;
        double errorShooterRPM = targetShooterRPM - currentShooterRPM;

        double targetVoltage;

        // --- LÓGICA DE CONTROL ---

        // A) BANG-BANG (Recuperación)
        if (isBangBangEnabled && errorShooterRPM > LowAltitudeConstants.SHOOTER_BANG_BANG_THRESHOLD) {
            targetVoltage = 14.0;
        }
        // B) COAST LOGIC (Frenado)
        else if (errorShooterRPM < -50) {
            targetVoltage = 0;
        }
        // C) PIDF (Fino)
        else {
            double ffVolts = feedforward.calculate(targetMotorRPM);
            double pidVolts = pidController.calculate(currentMotorRPM, targetMotorRPM);
            targetVoltage = ffVolts + pidVolts;
        }

        // --- CORRECCIÓN BATERÍA ---
        double batteryVoltage = batteryVoltageSensor.getVoltage();
        if (batteryVoltage <= 1.0) { // Si falla el sensor
            batteryVoltage = 12.5;
        }

        double finalPower = targetVoltage / batteryVoltage;

        // CLAMP AL 100%
        finalPower = Math.max(0.0, Math.min(1.0, finalPower));

        motorLeader.set(finalPower);
        motorFollower.set(finalPower);

        // Telemetría
        telemetry.addData("Shooter/Target", targetShooterRPM);
        telemetry.addData("Shooter/Actual", currentShooterRPM); // Debería mostrar 0 si hay glitch
        telemetry.addData("Shooter/Power", finalPower);
        telemetry.addData("Shooter/LoopCount", loopCycleCount);
    }
}