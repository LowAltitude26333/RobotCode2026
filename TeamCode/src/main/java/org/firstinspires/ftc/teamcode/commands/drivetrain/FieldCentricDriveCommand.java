package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotSafety;
import org.firstinspires.ftc.teamcode.commands.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import java.util.function.DoubleSupplier;

public class FieldCentricDriveCommand extends CommandBase {

    /**
     * Pedro prioriza giro cuando giro y traslacion llenan juntos el limite global.
     * Reservar parte del margen permite avanzar/strafe mientras el stick de giro
     * esta completamente inclinado.
     */
    public static final double TURN_MIX_SCALE = 0.70;

    private final PedroDriveSubsystem drive;
    private final DoubleSupplier strafeSup;
    private final DoubleSupplier forwardSup;
    private final DoubleSupplier turnSup;
    private boolean motionCommanded;

    public FieldCentricDriveCommand(PedroDriveSubsystem drive,
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
        double[] robotInput = fieldToRobot(strafe, forward, botHeading);

        // Fórmula: x' = x*cos(t) - y*sin(t) | y' = x*sin(t) + y*cos(t)

        // 3. Enviar al subsistema
        // The subsystem contract is strafe-right positive, matching GamepadEx left X.
        boolean zeroRequested = Math.abs(strafe) < 1e-6
                && Math.abs(forward) < 1e-6
                && Math.abs(turn) < 1e-6;
        if (motionCommanded && zeroRequested) {
            RobotSafety.timeZeroCommand("DRIVE_RELEASE", drive::stop);
        } else {
            drive.drive(robotInput[0], robotInput[1], turn);
        }
        motionCommanded = !zeroRequested;
    }

    /** Returns {strafeRight, forward} in the robot frame. */
    public static double[] fieldToRobot(double strafeRight, double forward,
                                        double botHeadingRadians) {
        // Driver strafe is right-positive, so this representation rotates by +heading.
        double theta = botHeadingRadians;
        double robotForward = forward * Math.cos(theta)
                - strafeRight * Math.sin(theta);
        double robotStrafeRight = forward * Math.sin(theta)
                + strafeRight * Math.cos(theta);
        return new double[] { robotStrafeRight, robotForward };
    }

    /** Stick derecho +derecha debe ordenar giro horario (turn neutral negativo). */
    public static double shapeDriverTurn(double driverRightStickX) {
        return -driverRightStickX * TURN_MIX_SCALE;
    }

    @Override
    public void end(boolean interrupted) {
        RobotSafety.timeZeroCommand(
                interrupted ? "DRIVE_INTERRUPTED" : "DRIVE_ENDED", drive::stop);
        motionCommanded = false;
    }
}
