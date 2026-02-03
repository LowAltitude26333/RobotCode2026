package org.firstinspires.ftc.teamcode.subsystems;

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
    public static final double TICKS_PER_REV = 28;

    private double targetRPM = 0;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        motorLeader = new MotorEx(hardwareMap, RobotMap.SHOOTER_MOTOR_1);
        motorFollower = new MotorEx(hardwareMap, RobotMap.SHOOTER_MOTOR_2);

        // Configuración de motores
        motorLeader.setInverted(RobotMap.SHOOTER_UP_MOTOR_IS_INVERTED);
        motorFollower.setInverted(RobotMap.SHOOTER_DOWN_MOTOR_IS_INVERTED);

        motorLeader.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motorFollower.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        motorLeader.setRunMode(Motor.RunMode.RawPower);
        motorFollower.setRunMode(Motor.RunMode.RawPower);

        motorLeader.resetEncoder();
        motorFollower.resetEncoder();

        pidfController = new PIDFController(
                LowAltitudeConstants.SHOOTER_KP,
                LowAltitudeConstants.SHOOTER_KI,
                LowAltitudeConstants.SHOOTER_KD,
                LowAltitudeConstants.SHOOTER_KF
        );
    }

    // --- MÉTODOS DE CONTROL ---

    public void setTargetRPM(double rpm) {
        // Reset del PID solo si venimos de 0 (arranque), para no perder inercia en ajustes finos
        if (this.targetRPM == 0 && rpm != 0) {
            pidfController.reset();
        }
        this.targetRPM = rpm;
    }

    // ¡MÉTODO RESTAURADO!
    // Permite pasar el Enum directamente desde los comandos
    public void setTargetEnum(LowAltitudeConstants.TargetRPM preset) {
        setTargetRPM(preset.targetRPM);
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getShooterRPM() {
        double motorTicksPerSec = motorLeader.getVelocity();
        double motorRPM = (motorTicksPerSec * 60.0) / LowAltitudeConstants.TICKS_PER_REV;
        return motorRPM * LowAltitudeConstants.SHOOTER_VELOCITY_MULTIPLIER;
    }

    public boolean onTarget() {
        return Math.abs(targetRPM - getShooterRPM()) < LowAltitudeConstants.RPM_OFFSET;
    }

    public void stop() {
        targetRPM = 0;
        motorLeader.set(0);
        motorFollower.set(0);
    }

    // Método para actualizar constantes desde Dashboard sin recompilar
    public void updatePIDCoefficients() {
        pidfController.setPIDF(
                LowAltitudeConstants.SHOOTER_KP,
                LowAltitudeConstants.SHOOTER_KI,
                LowAltitudeConstants.SHOOTER_KD,
                LowAltitudeConstants.SHOOTER_KF
        );
    }

    public void driveShooter(double power){
        motorLeader.set(power);
        motorFollower.set(power);
    }

    @Override
    public void periodic() {
        // Actualizar constantes PID en vivo (Solo para Tuning, comentar en competencia si se desea)
        updatePIDCoefficients();

        if (targetRPM == 0) {
            motorLeader.set(0);
            motorFollower.set(0);
            return;
        }

        double currentRPM = getShooterRPM();
        double error = targetRPM - currentRPM;
        double finalPower;

        // Lógica de Recuperación (Bang-Bang Híbrido)
        // IMPORTANTE: Asegúrate de corregir SHOOTER_RECOVERY_THRESHOLD en Constants
        if (error > LowAltitudeConstants.SHOOTER_RECOVERY_THRESHOLD) {
            finalPower = LowAltitudeConstants.SHOOTER_BOOST_SPEED;
            // Opcional: Resetear PID para evitar windup mientras recuperamos a fuerza bruta
            pidfController.reset();
        } else {
            // Control PIDF Fino
            double pidOutput = pidfController.calculate(currentRPM, targetRPM);
            finalPower = Math.max(-1.0, Math.min(pidOutput, 1.0));
        }

        motorLeader.set(finalPower);
        motorFollower.set(finalPower);

        // Telemetría útil para gráficas
        telemetry.addData("Shooter/Target", targetRPM);
        telemetry.addData("Shooter/Actual", currentRPM);
    }
}