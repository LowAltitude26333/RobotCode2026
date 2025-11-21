package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotMap;


public class ShooterSubsystem extends SubsystemBase {

    // 1. Declaración de Hardware y Constantes
    private final MotorEx motorLeader;
    private final MotorEx motorFollower;
    private final PIDFController pidfController;

    // Variables de estado
    private double currentTargetRPM = 0;
    private boolean onTarget = false;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        // Inicializar motores
        motorLeader = new MotorEx(hardwareMap, RobotMap.SHOOTER_MOTOR_1);
        motorFollower = new MotorEx(hardwareMap, RobotMap.SHOOTER_MOTOR_2);

        motorLeader.setInverted(RobotMap.SHOOTER_UP_MOTOR_IS_INVERTED);
        motorFollower.setInverted(RobotMap.SHOOTER_DOWN_MOTOR_IS_INVERTED);

        motorLeader.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motorFollower.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        motorLeader.setRunMode(Motor.RunMode.RawPower);
        motorFollower.setRunMode(Motor.RunMode.RawPower);

        motorLeader.resetEncoder();

        // Inicializar PIDFController
        // En FTCLib el PIDFController maneja el Feedforward automáticamente si se configura
        pidfController = new PIDFController(LowAltitudeConstants.SHOOTER_KP, LowAltitudeConstants.SHOOTER_KI,
                LowAltitudeConstants.SHOOTER_KD, LowAltitudeConstants.SHOOTER_KF);
    }

    /**
     * Mover el motor manualmente
     */
    public void driveShooter(double power) {
        motorLeader.set(power);
        motorFollower.set(power);
    }

    public void stopMotors() {
        driveShooter(0);
    }

    /**
     * Obtener Velocidad en RPM
     */
    public double getShooterRPM() {
        double rawVelocityTicksPerSec = motorLeader.getVelocity();
        // Conversión: (Ticks/Seg * 60) / Ticks_Por_Rev
        return (rawVelocityTicksPerSec * 60.0) /
                (LowAltitudeConstants.TICKS_PER_REV * LowAltitudeConstants.GEAR_RATIO);
    }

    public double getCurrentTarget() {
        return currentTargetRPM;
    }

    /**
     * Define el objetivo (Equivalente a setCurrentTarget)
     * @param targetRPM Velocidad deseada en RPM
     */
    public void setCurrentTarget(double targetRPM) {
        this.currentTargetRPM = targetRPM;
        // El setSetPoint se usa para el cálculo del término F
        pidfController.setSetPoint(targetRPM);
    }

    /**
     * Método principal de control
     * Calcula el PIDF y aplica potencia
     */
    public void maintainTarget(double maxPower) {
        // 1. Calcular Potencia PIDF
        // calculate(medición, setpoint)
        double calculatedPower = pidfController.calculate(getShooterRPM(), currentTargetRPM);

        // 2. Clamp (Restringir potencia entre -maxPower y maxPower)
        // Implementación manual de Math.clamp ya que Java 8 (Android antiguo) no siempre lo tiene nativo en Math
        double clampedPower = Math.max(-maxPower, Math.min(calculatedPower, maxPower));

        // 3. Aplicar potencia
        driveShooter(clampedPower);

        // 4. Verificar si estamos en el objetivo (OnTarget logic)
        double error = currentTargetRPM - getShooterRPM();
        this.onTarget = Math.abs(error) < LowAltitudeConstants.RPM_OFFSET;
    }

    public boolean onTarget() {
        return this.onTarget;
    }

    /**
     * Telemetría (Equivalente a putTuningValues)
     */
    public void putTuningValues(Telemetry telemetry) {
        telemetry.addData("Shooter Target RPM", getCurrentTarget());
        telemetry.addData("Shooter Actual RPM", getShooterRPM());
        telemetry.addData("Shooter Power", motorLeader.get());
        telemetry.addData("Shooter OnTarget", onTarget());
    }
}

/*
package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotMap;

public class ShooterSubsystem extends SubsystemBase {

    private final MotorEx motor1, motor2;
    private final SimpleMotorFeedforward feedforward;
    private double currentTargetRPM = 0;

    private final PIDFController pidfController;

    //RPM
    public double getShooterRPM() {
        double rawVelocityTicksPerSec = motor1.getVelocity();

        return (rawVelocityTicksPerSec * 60) / (LowAltitudeConstants.TICKS_PER_REV * LowAltitudeConstants.GEAR_RATIO);
    }

    public double getCurrentTarget() {
        return currentTargetRPM;
    }

    public ShooterSubsystem(HardwareMap hardwareMap, SimpleMotorFeedforward feedforward, PIDFController pidfController) {
        //Motores alv
        motor1 = new MotorEx(hardwareMap, RobotMap.SHOOTER_MOTOR_1);
        motor2 = new MotorEx(hardwareMap, RobotMap.SHOOTER_MOTOR_2);
        this.feedforward = feedforward;
        this.pidfController = pidfController;

        motor1.setRunMode(Motor.RunMode.RawPower);
        motor2.setRunMode(Motor.RunMode.RawPower);

        motor1.resetEncoder();
        motor2.resetEncoder();

        //PID gains
        PIDFController pidf = new PIDFController(
                LowAltitudeConstants.SHOOTER_KP,
                LowAltitudeConstants.SHOOTER_KI,
                LowAltitudeConstants.SHOOTER_KD,
                LowAltitudeConstants.SHOOTER_KF);
        }


    public void maintainTarget(double maxPower) {
        double calculatedPower = pidfController.calculate(getShooterRPM(), currentTargetRPM);

        double clampedPower = Math.max(-maxPower, Math.min(calculatedPower, maxPower));

        driveShooter(clampedPower);
    }

    public boolean onTarget() {
        return this.onTarget();
    }

    public void driveShooter(double power) {
        motor1.set(power);
        motor2.set(power);
    }

    public void shooterOn() {
        motor1.setVelocity(LowAltitudeConstants.SHOOTER_ON_SPEED);
        motor2.setVelocity(LowAltitudeConstants.SHOOTER_ON_SPEED);
    }

    public void shooterOff() {
        motor1.setVelocity(LowAltitudeConstants.SHOOTER_OFF_SPEED);
        motor2.setVelocity(LowAltitudeConstants.SHOOTER_OFF_SPEED);
    }

    public void shooterIntake() {
        motor1.setVelocity(LowAltitudeConstants.SHOOTER_INTAKE_SPEED);
        motor2.setVelocity(LowAltitudeConstants.SHOOTER_INTAKE_SPEED);
    }
}
*/
