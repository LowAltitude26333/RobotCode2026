package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseStorage {
    // Aquí se guarda la pose final del autónomo
    public static Pose2d currentPose = new Pose2d(0,0,0);

    public static boolean isRedAlliance = true;
    public static boolean isPrecisionMode = false;

    /**
     * Guarda el resultado de un autónomo Pedro Pathing para que MainTeleOp arranque desde ahí.
     * Toma doubles crudos (no un tipo de Pedro) porque vienen de
     * {@link org.firstinspires.ftc.teamcode.localization.PoseSnapshot}, ya agnóstico de localizador.
     */
    public static void recordAutonomousResult(double xInches, double yInches,
                                               double headingRadians, boolean redAlliance) {
        currentPose = new Pose2d(xInches, yInches, headingRadians);
        isRedAlliance = redAlliance;
    }
}
