package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LowAltitudeConstants {


        // --- SHOOTER --- Tuneado a 4500 RPM
        public static double SHOOTER_KP = 0.0008;//0.002; // Ajustar este valor es crucial
        public static double SHOOTER_KI = 0.0;
        public static double SHOOTER_KD = 0;//.0001;
        public static double SHOOTER_KF = 0.000155; // Feedforward para mantener velocidad

        public static double RPM_OFFSET = 50; // Tolerancia

        // Relación: 1 vuelta de motor = 2 vueltas de llanta
        // Esto significa que la llanta gira AL DOBLE de rápido que el motor.
        public static double SHOOTER_VELOCITY_MULTIPLIER = 2.0;
        public static double TICKS_PER_REV = 28.0; // Rev HD Hex Motor

        // Límite de potencia para no quemar motores o seguridad
        public static double SHOOTER_MAX_SPEED = 0.9; // Rango 0.0 a 1.0

        // --- SHOOTER HOOD ---
        // Estos son los ángulos físicos que permitimos por software
        public static final double HOOD_MIN_LIMIT = 5.0;
        public static final double HOOD_MAX_LIMIT = 207.0;

        // Estos son los límites físicos del servo (0 a 300 en goBILDA, 0 a 270 en REV)
    public static double HOOD_SERVO_MAX_RANGE = 300.0;

    public static final double INTAKE_IN_SPEED = 0.5;
    public static final double INTAKE_STOP = 0.0;
    public static final double INTAKE_REVERSE = -0.5;

    public enum HoodPosition {
        WALL_SHOT(5.0),    // Tiro cercano
        MID_FIELD(106.0),    // Tiro medio
        LONG_SHOT(207.0);    // Tiro lejano

        public final double angle;

        HoodPosition(double angle) {
            this.angle = angle;
        }
    }

    // --- KICKER CONSTANTS ---
    public static final double KICKER_OUT_SPEED = 0.5;
    public static final double KICKER_REVERSE_SPEED = -0.5;

}
