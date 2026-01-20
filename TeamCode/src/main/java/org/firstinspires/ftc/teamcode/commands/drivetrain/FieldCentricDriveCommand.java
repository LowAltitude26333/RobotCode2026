package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class FieldCentricDriveCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier forwardSup;
    private final DoubleSupplier turnSup;

    public FieldCentricDriveCommand(DriveSubsystem drive,
                                    DoubleSupplier strafeSup,
                                    DoubleSupplier forwardSup,
                                    DoubleSupplier turnSup) {
        this.drive = drive;
        this.strafeSup = strafeSup;
        this.forwardSup = forwardSup;
        this.turnSup = turnSup;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        // 1. Obtener inputs del joystick
        double strafe = strafeSup.getAsDouble();
        double forward = forwardSup.getAsDouble();
        double turn = turnSup.getAsDouble();

        // 2. Obtener el ángulo del robot
        double botHeading = drive.getHeading();

        /* * FIX POWERHOUSE: MATEMÁTICA MANUAL
         * Como Vector2d.rotated() no existe en Java RR 1.0, usamos trigonometría pura.
         * Fórmulas de rotación 2D estándar.
         */

        // Alinear inputs con el sistema de coordenadas de RoadRunner (X=Forward, Y=Left)
        double x = -forward;
        double y = strafe; // Invertimos strafe porque RR usa Y+ a la izquierda

        // Rotamos por el negativo del heading para hacer "Field Centric"
        double theta = -botHeading;

        // Fórmula: x' = x*cos(t) - y*sin(t) | y' = x*sin(t) + y*cos(t)
        double rotX = x * Math.cos(theta) - y * Math.sin(theta);
        double rotY = x * Math.sin(theta) + y * Math.cos(theta);

        // 3. Enviar al subsistema
        // El subsistema espera (strafe, forward, turn)
        // rotY es nuestro nuevo Strafe (Left+), rotX es nuestro nuevo Forward
        // Invertimos rotY al enviarlo porque tu drive() probablemente espera Strafe Right+
        drive.drive(-rotY, rotX, turn);
    }
}