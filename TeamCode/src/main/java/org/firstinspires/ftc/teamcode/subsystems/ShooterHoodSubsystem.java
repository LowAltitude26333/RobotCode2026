package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants; // Importamos las constantes
import org.firstinspires.ftc.teamcode.RobotMap;

public class ShooterHoodSubsystem extends SubsystemBase {

    private final ServoEx leftServo;
    private final ServoEx rightServo;

    public ShooterHoodSubsystem(HardwareMap hardwareMap) {
        // Configuración de servos duales
        leftServo = new SimpleServo(
                hardwareMap, RobotMap.HOOD_SERVO_LEFT,
                LowAltitudeConstants.HOOD_MIN_ANGLE_DEG,
                LowAltitudeConstants.HOOD_MAX_ANGLE_DEG
        );

        rightServo = new SimpleServo(
                hardwareMap, RobotMap.HOOD_SERVO_RIGHT,
                LowAltitudeConstants.HOOD_MIN_ANGLE_DEG,
                LowAltitudeConstants.HOOD_MAX_ANGLE_DEG
        );

        leftServo.setInverted(RobotMap.HOOD_LEFT_INVERTED);
        rightServo.setInverted(RobotMap.HOOD_RIGHT_INVERTED);
    }

    public void setAngle(double degrees) {
        double safeAngle = Math.max(
                LowAltitudeConstants.HOOD_MIN_LIMIT,
                Math.min(degrees, LowAltitudeConstants.HOOD_MAX_LIMIT)
        );

        leftServo.turnToAngle(safeAngle);
        rightServo.turnToAngle(safeAngle);
    }

    /**
     * Método actualizado: Recibe el Enum desde LowAltitudeConstants
     */
    public void setPosition(LowAltitudeConstants.HoodPosition position) {
        setAngle(position.angle);
    }

    public double getAngle() {
        return leftServo.getAngle();
    }
}