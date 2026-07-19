package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotSafety;
import org.firstinspires.ftc.teamcode.util.Angles;

public class DriveSubsystem extends SubsystemBase {

    private final MecanumDrive drive;
    private final Telemetry telemetry;
    private double headingOffsetRadians;
    private PoseVelocity2d lastVelocity;
    private int resetEpoch;
    private boolean poseEverUpdated;

    public DriveSubsystem(HardwareMap hardwareMap, Pose2d startPose, Telemetry telemetry) {
        // Inicializa RoadRunner (esto ya configura los motores internamente)
        this.drive = new MecanumDrive(hardwareMap, startPose);
        this.telemetry = telemetry;
        stop();
        RobotSafety.registerShutdown(this::stop);
    }

    /**
     * Mueve el robot con velocidades relativas al ROBOT.
     * La conversión a Field Centric ya ocurrió en el COMANDO antes de llegar aquí.
     *
     * @param strafeSpeed Velocidad lateral (Derecha +)
     * @param forwardSpeed Velocidad frontal (Adelante +)
     * @param turnSpeed Velocidad de giro (Izquierda +)
     */
    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {

        // RoadRunner 1.0 usa un sistema de coordenadas diferente:
        // Vector2d(X, Y) -> X es Adelante/Atrás, Y es Izquierda/Derecha (Strafe)
        // Por eso pasamos (forward, -strafe)

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(forwardSpeed, -strafeSpeed),
                -turnSpeed
        ));

        // Debug opcional
        // telemetry.addData("Powers", "Fwd: %.2f, Str: %.2f, Turn: %.2f", forwardSpeed, strafeSpeed, turnSpeed);
    }

    /*public void resetHeading() {
        // En RoadRunner 1.0, modificamos la pose actual manteniendo X e Y, pero forzando Heading a 0
        Pose2d currentPose = drive.pose;
        drive.pose = new Pose2d(currentPose.position.x, currentPose.position.y, 0);
    }*/

    public void stop() {
        drive(0, 0, 0);
    }

    // Método vital para usar en Autonómo si necesitas acceder a la instancia raw
    public MecanumDrive getMecanumDrive() {
        return drive;
    }

    public double getHeading() {
        double rawHeading = drive.lazyImu.get().getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);
        return normalizeRadians(rawHeading - headingOffsetRadians);
    }

    public void resetHeading() {
        headingOffsetRadians = drive.lazyImu.get().getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);

        // 1. Obtener la pose actual
        Pose2d currentPose = drive.pose;

        // 2. Crear una nueva pose con las mismas coordenadas X, Y, pero Heading = 0
        Pose2d newPose = new Pose2d(currentPose.position.x, currentPose.position.y, 0);

        // 3. CRÍTICO: Decirle al localizer que esta es la nueva verdad
        drive.localizer.setPose(newPose); // <--- ESTO FALTABA

        // 4. Actualizar la variable local también por si acaso
        drive.pose = newPose;

        // La pose saltó a propósito: los consumidores de PoseProvider deben
        // descartar continuidad al ver cambiar el epoch.
        resetEpoch++;
    }

    /** Reposiciona la pose completa (usado por el PoseProvider neutral de MP-02). */
    public void setPose(Pose2d newPose) {
        drive.localizer.setPose(newPose);
        drive.pose = newPose;
        resetEpoch++;
    }

    public int getResetEpoch() {
        return resetEpoch;
    }

    /** Última velocidad de updatePoseEstimate(); null hasta el primer periodic(). */
    public PoseVelocity2d getLastVelocity() {
        return lastVelocity;
    }

    /** true después del primer ciclo de odometría (gate de UNINITIALIZED). */
    public boolean isPoseEverUpdated() {
        return poseEverUpdated;
    }

    private static double normalizeRadians(double angle) {
        return Angles.normalizeRadians(angle);
    }

    @Override
    public void periodic() {
        // CRÍTICO: Esto mantiene la odometría viva durante TeleOp.
        // Si no llamas a esto, el robot "pierde" su posición en el campo.
        lastVelocity = drive.updatePoseEstimate();
        poseEverUpdated = true;

        // Telemetría útil para el driver
        Pose2d p = drive.pose;
        telemetry.addData("Pose", "X: %.1f, Y: %.1f, H: %.1f°",
                p.position.x,
                p.position.y,
                Math.toDegrees(p.heading.toDouble()));
    }
}
