package org.firstinspires.ftc.teamcode.localization;

/**
 * Comando de movimiento neutral (MP-02): los consumidores ordenan velocidades
 * robot-céntricas sin importar qué stack (Road Runner hoy, Pedro con DEC-034)
 * las ejecuta. Convención idéntica a DriveSubsystem.drive():
 * strafe +derecha, forward +adelante, turn +izquierda, todos en [-1, 1].
 */
public interface DriveAdapter {

    void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed);

    /** Cero inmediato e idempotente; debe ser seguro como hook de stop. */
    void stop();
}
