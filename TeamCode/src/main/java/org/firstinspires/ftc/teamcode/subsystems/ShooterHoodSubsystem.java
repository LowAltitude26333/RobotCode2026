package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotMap;

public class ShooterHoodSubsystem extends SubsystemBase {

    private final ServoEx leftServo;
    private final ServoEx rightServo;
    private final Telemetry telemetry;

    public ShooterHoodSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Configuramos el rango físico TOTAL del servo (ej. 0 a 300 grados)
        leftServo = new SimpleServo(
                hardwareMap, RobotMap.HOOD_SERVO_LEFT,
                0, LowAltitudeConstants.HOOD_SERVO_MAX_RANGE
        );

        rightServo = new SimpleServo(
                hardwareMap, RobotMap.HOOD_SERVO_RIGHT,
                0, LowAltitudeConstants.HOOD_SERVO_MAX_RANGE
        );

        leftServo.setInverted(RobotMap.HOOD_LEFT_INVERTED);
        rightServo.setInverted(RobotMap.HOOD_RIGHT_INVERTED);
    }

    public void setPosition(LowAltitudeConstants.HoodPosition position) {
        setAngle(position.angle);
    }

    public void setAngle(double degrees) {
        // Clamp de seguridad con tus nuevos límites (25 a 55)
        double safeAngle = Math.max(
                LowAltitudeConstants.HOOD_MIN_LIMIT,
                Math.min(degrees, LowAltitudeConstants.HOOD_MAX_LIMIT)
        );

        leftServo.turnToAngle(safeAngle);
        rightServo.turnToAngle(safeAngle);
    }

    // Método para debug: Mover por porcentaje (0.0 a 1.0) para encontrar los ángulos reales
    public void setRawPosition(double position) {
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }

    @Override
    public void periodic() {
        // Imprimimos la posición lógica (Grados) y la cruda (0-1)
        telemetry.addData("HOOD Angle Command", leftServo.getAngle());
        telemetry.addData("HOOD Raw Position", leftServo.getPosition());
    }
}