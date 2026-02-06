package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LowAltitudeConstants {

    public enum TargetRPM {
        WALL_SHOT_RPM(2450.0),    // Tiro cercano + 2750 RPM Target + 17 inches2450
        SHORT_SHOT_RPM(2900.0), // Tiro en medio medio de la cancha + 2850 RPM Target + 36 inches2550
        MID_FIELD_RPM(2900.0),    // Tiro media cancha + 3300 RPM Target3000
        LONG_SHOT_RPM(3600.0);    // Tiro full court + 3625 RPM Target3425

        public final double targetRPM;

        TargetRPM(double targetRPM) {
            this.targetRPM = targetRPM;
        }
    }

    // --- SHOOTER HARDWARE ---
    // Ajusta esto según tu motor exacto (Ej. GoBilda 5203 1:1)
        public static double MOTOR_TICKS_PER_REV = 28.0;
        public static double MOTOR_MAX_RPM = 6000.0;
        public static double SHOOTER_TOLERANCE_RPM = 50.0; // Rango aceptable para disparar

        // --- SHOOTER CONTROL (FEEDFORWARD) ---
        // kS: Voltaje estático (fricción mínima para empezar a mover).
        public static double SHOOTER_KS = 2.5;
        // kV: Voltios necesarios por cada RPM. (Ej. 12V / 6000RPM = 0.002)
        public static double SHOOTER_KV = 0.003;
        // kA: Aceleración (opcional, dejamos en 0 para shooter velocidad constante).
        public static double SHOOTER_KA = 0.0;

        // --- SHOOTER CONTROL (PID) --

    // KP: Corrección de error. Empieza muy bajo (0.0001) porque Feedforward hace el 90%.
        public static double SHOOTER_KP = 0.00075;
        public static double SHOOTER_KI = 0.0;
        public static double SHOOTER_KD = 0.0;

        // --- SHOOTER LOGIC ---
        // Si el error es mayor a esto (ej. al disparar), ignora PID y da 100% de potencia.
        public static double SHOOTER_BANG_BANG_THRESHOLD = 150.0;
        // Voltaje nominal para cálculos (batería llena teórica).
        public static double NOMINAL_VOLTAGE = 12.0;

    // Relación Externa: 2.0 significa que el Shooter gira 2 veces más rápido que el motor
    // (Polea Grande en Motor -> Polea Pequeña en Shooter)
    public static double SHOOTER_GEAR_RATIO = 2.0;

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
        WALL_SHOT(202.0),    // Tiro cercano + 2450 RPM Target + 17 inches 202
        SHORT_SHOT(170.0), // Tiro en medio medio de la cancha + 2550 RPM Target + 36 inches 127
        MID_FIELD(127.0),    // Tiro media cancha + 3000 RPM Target 127
        LONG_SHOT(62.0),    // Tiro full court + 3425 RPM Target 62
        HOME_POS( 0.0);

        public final double angle;

        HoodPosition(double angle) {
            this.angle = angle;
        }
    }

    // --- KICKER CONSTANTS ---
    public static final double KICKER_OUT_SPEED = 0.7;
    public static final double KICKER_REVERSE_SPEED = -0.7;

}