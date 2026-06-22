package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.RobotSafety;

public class TurretSubsystem extends SubsystemBase {
    private final DcMotor turretMotor;
    private boolean centeredAndArmed;

    // --- CONFIGURACIÓN DE LÍMITES (CALIBRADOS) ---
    // IMPORTANTE: Ajusta estos números según lo que anotaste en tus pruebas
    public static final int LIMIT_LEFT = -200;
    public static final int LIMIT_RIGHT = 200;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, RobotMap.TORRETA_MOTOR);

        // Aplicar inversión desde el RobotMap para que el encoder y el motor coincidan
        if (RobotMap.TORRETA_IS_INVERTED) {
            turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // BRAKE ayuda a que la torreta no se pase del límite por inercia
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setPower(0);
        centeredAndArmed = false;
        RobotSafety.registerShutdown(this::disarm);
    }

    public void confirmCenteredAndResetEncoder() {
        stop();
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        centeredAndArmed = true;
    }

    public boolean isArmed() {
        return centeredAndArmed;
    }

    public void disarm() {
        centeredAndArmed = false;
        stop();
    }

    /**
     * Establece la potencia del motor verificando los límites del encoder.
     * @param power Potencia solicitada (-1.0 a 1.0)
     */
    public void setPower(double power) {
        if (!centeredAndArmed) {
            turretMotor.setPower(0);
            return;
        }

        int currentPos = (int) getPosition();

        // BLOQUEO DE SEGURIDAD
        // Si intenta ir a la izquierda (potencia negativa) y ya pasó el límite izquierdo
        if (currentPos <= LIMIT_LEFT && power < 0) {
            turretMotor.setPower(0);
        }
        // Si intenta ir a la derecha (potencia positiva) y ya pasó el límite derecho
        else if (currentPos >= LIMIT_RIGHT && power > 0) {
            turretMotor.setPower(0);
        }
        // De lo contrario, movimiento libre
        else {
            turretMotor.setPower(power);
        }
    }

    public double getPosition() {
        return turretMotor.getCurrentPosition();
    }

    public void stop() {
        turretMotor.setPower(0);
    }

    @Override
    public void periodic() {
        // Esta parte se ejecuta todo el tiempo. Útil para debug.
        // Si usas Telemetry en el OpMode, puedes borrar esto.
    }
}
