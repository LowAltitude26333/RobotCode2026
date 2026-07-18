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
import org.firstinspires.ftc.teamcode.RobotSafety;

public class ShooterSubsystem extends SubsystemBase {

    public enum HealthState {
        UNINITIALIZED,
        HEALTHY,
        ENCODER_FAULT,
        VOLTAGE_FAULT,
        OVERSPEED,
        INVALID_TARGET,
        CONTROL_FAULT
    }

    public enum FaultInjectionMode {
        NONE,
        INVALID_VOLTAGE,
        ENCODER_FROZEN,
        RPM_NON_FINITE,
        OVERSPEED
    }

    // 1. UN SOLO MOTOR (Dejamos únicamente al líder)
    private final MotorEx motorLeader;
    private final VoltageSensor batteryVoltageSensor;
    private final Telemetry telemetry;

    private PIDController pidController;
    private SimpleMotorFeedforward feedforward;
    private double loadedKp;
    private double loadedKi;
    private double loadedKd;
    private double loadedKs;
    private double loadedKv;
    private double loadedKa;

    private double targetShooterRPM = 0;
    private boolean isBangBangEnabled = true;

    // --- Variables de Seguridad ---
    private int loopCycleCount = 0;
    private double actualShooterRPM;
    private double lastAppliedPower;
    private int lastEncoderPosition;
    private boolean encoderPositionInitialized;
    private long encoderStallStartedNanos;
    private HealthState healthState = HealthState.UNINITIALIZED;
    private String faultReason = "warming up";
    private FaultInjectionMode faultInjectionMode = FaultInjectionMode.NONE;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        VoltageSensor configuredVoltageSensor = null;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            configuredVoltageSensor = sensor;
            break;
        }
        batteryVoltageSensor = configuredVoltageSensor;

        // Inicialización del Motor Único
        motorLeader = new MotorEx(hardwareMap, RobotMap.SHOOTER_MOTOR,
                LowAltitudeConstants.MOTOR_TICKS_PER_REV,
                LowAltitudeConstants.MOTOR_MAX_RPM);

        configureMotors();
        applyPower(0);
        reloadControllers();
        RobotSafety.registerShutdown(this::stop);
    }

    private void configureMotors() {
        // 1. Invertir Motor si es necesario
        motorLeader.setInverted(RobotMap.SHOOTER_MOTOR_IS_INVERTED);

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
        loadedKp = LowAltitudeConstants.SHOOTER_KP;
        loadedKi = LowAltitudeConstants.SHOOTER_KI;
        loadedKd = LowAltitudeConstants.SHOOTER_KD;
        loadedKs = LowAltitudeConstants.SHOOTER_KS;
        loadedKv = LowAltitudeConstants.SHOOTER_KV;
        loadedKa = LowAltitudeConstants.SHOOTER_KA;
    }

    public void reloadControllersIfConstantsChanged() {
        if (Double.compare(loadedKp, LowAltitudeConstants.SHOOTER_KP) != 0
                || Double.compare(loadedKi, LowAltitudeConstants.SHOOTER_KI) != 0
                || Double.compare(loadedKd, LowAltitudeConstants.SHOOTER_KD) != 0
                || Double.compare(loadedKs, LowAltitudeConstants.SHOOTER_KS) != 0
                || Double.compare(loadedKv, LowAltitudeConstants.SHOOTER_KV) != 0
                || Double.compare(loadedKa, LowAltitudeConstants.SHOOTER_KA) != 0) {
            reloadControllers();
        }
    }

    public void setTargetRPM(double shooterRPM) {
        if (isFaultLatched()) {
            return;
        }
        if (!Double.isFinite(shooterRPM)
                || shooterRPM < 0
                || shooterRPM > LowAltitudeConstants.SHOOTER_MAX_SAFE_RPM) {
            latchFault(HealthState.INVALID_TARGET,
                    "target RPM outside 0.." + LowAltitudeConstants.SHOOTER_MAX_SAFE_RPM);
            return;
        }
        if (Math.abs(shooterRPM - targetShooterRPM) > 500) {
            pidController.reset();
        }
        this.targetShooterRPM = shooterRPM;
        if (shooterRPM == 0) {
            applyPower(0);
            encoderStallStartedNanos = 0;
        }
    }

    public void setTargetEnum(LowAltitudeConstants.TargetRPM preset) {
        setTargetRPM(preset.targetRPM);
    }

    public double getTargetRPM() {
        return targetShooterRPM;
    }

    public HealthState getHealthState() {
        return healthState;
    }

    public String getFaultReason() {
        return faultReason;
    }

    public double getBatteryVoltage() {
        return readBatteryVoltage();
    }

    public boolean isHealthy() {
        return healthState == HealthState.HEALTHY;
    }

    public boolean isFaultLatched() {
        return healthState != HealthState.UNINITIALIZED
                && healthState != HealthState.HEALTHY;
    }

    public FaultInjectionMode getFaultInjectionMode() {
        return faultInjectionMode;
    }

    public double getLastAppliedPower() {
        return lastAppliedPower;
    }

    /** Commissioning-only injection. Always commands zero before arming the simulated fault. */
    public boolean requestFaultInjection(FaultInjectionMode requestedMode) {
        if (requestedMode == null
                || requestedMode == FaultInjectionMode.NONE
                || isFaultLatched()) {
            return false;
        }

        stop();
        faultInjectionMode = requestedMode;
        return true;
    }

    public void stop() {
        targetShooterRPM = 0;
        encoderStallStartedNanos = 0;
        applyPower(0);
    }

    /**
     * Devuelve la velocidad real FILTRADA usando solo el encoder del motor líder.
     */
    public double getActualShooterRPM() {
        return actualShooterRPM;
    }

    public boolean isReady() {
        return isHealthy() && targetShooterRPM > 0 &&
                Math.abs(targetShooterRPM - getActualShooterRPM()) <= LowAltitudeConstants.SHOOTER_TOLERANCE_RPM;
    }

    @Override
    public void periodic() {
        loopCycleCount++;

        if (isFaultLatched()) {
            applyPower(0);
            publishTelemetry(Double.NaN);
            return;
        }

        double batteryVoltage = faultInjectionMode == FaultInjectionMode.INVALID_VOLTAGE
                ? Double.NaN
                : readBatteryVoltage();
        if (!Double.isFinite(batteryVoltage) || batteryVoltage <= 1.0) {
            latchFault(HealthState.VOLTAGE_FAULT, "invalid or unavailable battery voltage");
            publishTelemetry(batteryVoltage);
            return;
        }

        if (faultInjectionMode == FaultInjectionMode.ENCODER_FROZEN) {
            latchFault(HealthState.ENCODER_FAULT, "injected frozen encoder");
            publishTelemetry(batteryVoltage);
            return;
        }

        double currentShooterRPM;
        if (faultInjectionMode == FaultInjectionMode.RPM_NON_FINITE) {
            currentShooterRPM = Double.NaN;
        } else if (faultInjectionMode == FaultInjectionMode.OVERSPEED) {
            currentShooterRPM = LowAltitudeConstants.SHOOTER_MAX_SAFE_RPM + 1.0;
        } else {
            try {
                double motorVelocityTicks = motorLeader.getCorrectedVelocity();
                double motorRPM = (motorVelocityTicks / LowAltitudeConstants.MOTOR_TICKS_PER_REV) * 60.0;
                currentShooterRPM = motorRPM * LowAltitudeConstants.SHOOTER_GEAR_RATIO;
            } catch (RuntimeException exception) {
                latchFault(HealthState.ENCODER_FAULT, "encoder read failed");
                publishTelemetry(batteryVoltage);
                return;
            }
        }

        if (!Double.isFinite(currentShooterRPM)) {
            latchFault(HealthState.ENCODER_FAULT, "non-finite encoder velocity");
            publishTelemetry(batteryVoltage);
            return;
        }
        if (Math.abs(currentShooterRPM) > LowAltitudeConstants.SHOOTER_MAX_SAFE_RPM) {
            latchFault(HealthState.OVERSPEED,
                    "measured RPM exceeds " + LowAltitudeConstants.SHOOTER_MAX_SAFE_RPM);
            publishTelemetry(batteryVoltage);
            return;
        }

        actualShooterRPM = currentShooterRPM;

        // Keep the motor off while FTCLib velocity correction initializes.
        if (loopCycleCount < 30) {
            healthState = HealthState.UNINITIALIZED;
            faultReason = "warming up";
            applyPower(0);
            publishTelemetry(batteryVoltage);
            return;
        }

        healthState = HealthState.HEALTHY;
        faultReason = "none";

        // Si el objetivo es 0, apagamos por completo el motor y salimos
        if (targetShooterRPM == 0) {
            applyPower(0);
            encoderStallStartedNanos = 0;
            publishTelemetry(batteryVoltage);
            return;
        }

        monitorEncoderResponse(System.nanoTime());
        if (isFaultLatched()) {
            publishTelemetry(batteryVoltage);
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

        double finalPower = targetVoltage / batteryVoltage;

        if (!Double.isFinite(finalPower)) {
            latchFault(HealthState.CONTROL_FAULT, "non-finite PID/feedforward output");
            publishTelemetry(batteryVoltage);
            return;
        }

        // CLAMP DE SEGURIDAD (0.0 a 1.0)
        finalPower = Math.max(0.0, Math.min(1.0, finalPower));

        // Aplicar la potencia calculada al único motor activo
        applyPower(finalPower);

        publishTelemetry(batteryVoltage);
    }

    private double readBatteryVoltage() {
        if (batteryVoltageSensor == null) {
            return Double.NaN;
        }
        try {
            return batteryVoltageSensor.getVoltage();
        } catch (RuntimeException exception) {
            return Double.NaN;
        }
    }

    private void monitorEncoderResponse(long nowNanos) {
        int encoderPosition;
        try {
            encoderPosition = motorLeader.motor.getCurrentPosition();
        } catch (RuntimeException exception) {
            latchFault(HealthState.ENCODER_FAULT, "encoder position read failed");
            return;
        }

        if (!encoderPositionInitialized || encoderPosition != lastEncoderPosition) {
            encoderPositionInitialized = true;
            lastEncoderPosition = encoderPosition;
            encoderStallStartedNanos = 0;
            return;
        }

        if (lastAppliedPower <= 0 || targetShooterRPM <= 0) {
            encoderStallStartedNanos = 0;
            return;
        }

        if (encoderStallStartedNanos == 0) {
            encoderStallStartedNanos = nowNanos;
            return;
        }

        long timeoutNanos = Math.max(1, LowAltitudeConstants.SHOOTER_READY_TIMEOUT_MS)
                * 1_000_000L;
        if (nowNanos - encoderStallStartedNanos >= timeoutNanos) {
            latchFault(HealthState.ENCODER_FAULT, "encoder did not change before timeout");
        }
    }

    private void latchFault(HealthState fault, String reason) {
        if (isFaultLatched()) {
            return;
        }
        healthState = fault;
        faultReason = reason;
        targetShooterRPM = 0;
        applyPower(0);
    }

    private void applyPower(double power) {
        lastAppliedPower = power;
        motorLeader.set(power);
    }

    private void publishTelemetry(double batteryVoltage) {
        telemetry.addData("Shooter/Health", healthState);
        telemetry.addData("Shooter/Fault", faultReason);
        telemetry.addData("Shooter/BatteryV", batteryVoltage);

        // Telemetría de diagnóstico
        telemetry.addData("Shooter/Target", targetShooterRPM);
        telemetry.addData("Shooter/Actual", actualShooterRPM);
        telemetry.addData("Shooter/Power", lastAppliedPower);
        telemetry.addData("Shooter/LoopCount", loopCycleCount);
        telemetry.addData("Shooter/Injected", faultInjectionMode);
    }
}
