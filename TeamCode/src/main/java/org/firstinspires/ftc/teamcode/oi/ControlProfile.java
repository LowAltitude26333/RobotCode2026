package org.firstinspires.ftc.teamcode.oi;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Interfaz que define cómo debe comportarse cualquier configuración de controles.
 * Permite tener "DriverAProfile", "DriverBProfile", "TestProfile", etc.
 */
public interface ControlProfile {

    // Métodos para obtener los Gamepads (útil si necesitas leer ejes crudos)
    GamepadEx getDriverOp();
    GamepadEx getToolOp();

    // Métodos para el manejo del chasis (Suppliers)
    double getDriveStrafe();
    double getDriveForward();
    double getDriveTurn();

    /**
     * Aquí es donde "enchufamos" los botones a los comandos del robot.
     * @param robot Referencia al contenedor para acceder a los subsistemas.
     */
    void configureButtonBindings(RobotContainer robot);
}