package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotSafety;
import org.firstinspires.ftc.teamcode.commands.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class FieldCentricDriveCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier forwardSup;
    private final DoubleSupplier turnSup;
    private boolean motionCommanded;

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

        if (PoseStorage.isPrecisionMode) {
             strafe = strafeSup.getAsDouble() * 0.5;
             forward = forwardSup.getAsDouble() * 0.5;
             turn = turnSup.getAsDouble() * 0.3;
        }


        // 2. Obtener el ángulo del robot
        double botHeading = drive.getHeading();

        /* * FIX POWERHOUSE: MATEMÁTICA MANUAL
         * Como Vector2d.rotated() no existe en Java RR 1.0, usamos trigonometría pura.
         * Fórmulas de rotación 2D estándar.
         */

        // FTCLib GamepadEx already reports left-stick Y as positive when pushed forward.
        // Road Runner also uses X+ as forward, so no additional sign inversion belongs here.
        double x = forward;
        double y = strafe; // Invertimos strafe porque RR usa Y+ a la izquierda

        // Rotamos por el negativo del heading para hacer "Field Centric"
        double theta = -botHeading;

        // Fórmula: x' = x*cos(t) - y*sin(t) | y' = x*sin(t) + y*cos(t)
        double rotX = x * Math.cos(theta) - y * Math.sin(theta);
        double rotY = x * Math.sin(theta) + y * Math.cos(theta);

        // 3. Enviar al subsistema
        // The subsystem contract is strafe-right positive, matching GamepadEx left X.
        boolean zeroRequested = Math.abs(strafe) < 1e-6
                && Math.abs(forward) < 1e-6
                && Math.abs(turn) < 1e-6;
        if (motionCommanded && zeroRequested) {
            RobotSafety.timeZeroCommand("DRIVE_RELEASE", drive::stop);
        } else {
            drive.drive(rotY, rotX, turn);
        }
        motionCommanded = !zeroRequested;
    }

    @Override
    public void end(boolean interrupted) {
        RobotSafety.timeZeroCommand(
                interrupted ? "DRIVE_INTERRUPTED" : "DRIVE_ENDED", drive::stop);
        motionCommanded = false;
    }
}
