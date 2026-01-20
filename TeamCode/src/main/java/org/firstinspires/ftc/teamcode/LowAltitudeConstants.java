package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LowAltitudeConstants {


    // --- SHOOTER --- Tuneado a 4500 RPM
    public static double SHOOTER_KP = 0.0015; // Ajustar este valor es crucial
    public static double SHOOTER_KI = 0.000;
    public static double SHOOTER_KD = 0.0001;
    public static double SHOOTER_KF = 0.000229 ; // Feedforward para mantener velocidad

    // NUEVO: Umbral de recuperación agresiva
    // Si la velocidad cae más de 150 RPM del target, aplicamos fuerza bruta.
    public static double SHOOTER_RECOVERY_THRESHOLD = -5; //Antes 150. Cambiado ya que al llegar al error, dejaba de dar más poder.

    public static double RPM_OFFSET = 10; // Tolerancia Anterior 100


    public enum TargetRPM {
        WALL_SHOT_RPM(2450.0),    // Tiro cercano + 2750 RPM Target + 17 inches
        SHORT_SHOT_RPM(2550.0), // Tiro en medio medio de la cancha + 2850 RPM Target + 36 inches
        MID_FIELD_RPM(3000.0),    // Tiro media cancha + 3300 RPM Target
        LONG_SHOT_RPM(3425.0);    // Tiro full court + 3625 RPM Target

        public final double targetRPM;

        TargetRPM(double targetRPM) {
            this.targetRPM = targetRPM;
        }
    }

    // Relación: 1 vuelta de motor = 2 vueltas de llanta
    // Esto significa que la llanta gira AL DOBLE de rápido que el motor.
    public static double SHOOTER_VELOCITY_MULTIPLIER = 2.0;
    public static double TICKS_PER_REV = 28.0; // Rev HD Hex Motor

    // Límite de potencia para no quemar motores o seguridad
    public static double SHOOTER_MAX_SPEED = 0.9; // Rango 0.0 a 1.0

    // NUEVO: Límite exclusivo para emergencias (Recuperación)
    public static double SHOOTER_BOOST_SPEED = 1.0;

    // --- SHOOTER HOOD ---
    // Estos son los ángulos físicos que permitimos por software
    public static final double HOOD_MIN_LIMIT = 5.0;
    public static final double HOOD_MAX_LIMIT = 207.0;

    // Estos son los límites físicos del servo (0 a 300 en goBILDA, 0 a 270 en REV)
    public static double HOOD_SERVO_MAX_RANGE = 300.0;

    public static final double INTAKE_IN_SPEED = 0.6;
    public static final double INTAKE_STOP = 0.0;
    public static final double INTAKE_REVERSE = -0.5;

    public enum HoodPosition {
        WALL_SHOT(202.0),    // Tiro cercano + 2750 RPM Target + 17 inches
        SHORT_SHOT(127.0), // Tiro en medio medio de la cancha + 2850 RPM Target + 36 inches
        MID_FIELD(127.0),    // Tiro media cancha + 3300 RPM Target
        LONG_SHOT(62.0),    // Tiro full court + 3625 RPM Target
        HOME_POS( 0.0);

        public final double angle;

        HoodPosition(double angle) {
            this.angle = angle;
        }
    }

    // --- KICKER CONSTANTS ---
    public static final double KICKER_OUT_SPEED = 0.8;
    public static final double KICKER_REVERSE_SPEED = -0.5;

}