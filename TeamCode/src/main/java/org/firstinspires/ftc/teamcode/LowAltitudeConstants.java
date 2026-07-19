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
        public static final double SHOOTER_MAX_SAFE_RPM = 6000.0;
        public static double SHOOTER_TOLERANCE_RPM = 90.0; // Rango aceptable para disparar

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

    public static long SHOOTER_READY_TIMEOUT_MS = 2000;
    public static int SHOOTER_READY_MAX_ATTEMPTS = 3;

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

        SHORT_SHOT(155.0), // Tiro en medio medio de la cancha + 2550 RPM Target + 36 inches 127
        MID_FIELD(127.0),    // Tiro media cancha + 3000 RPM Target 127
        LONG_SHOT(62.0),    // Tiro full court + 3425 RPM Target 62
        HOME_POS( 0.0);

        public final double angle;

        HoodPosition(double angle) {
            this.angle = angle;
        }
    }

    // --- KICKER CONSTANTS ---
    // Dashboard toggle. Change only while the OpMode is stopped; it is captured at INIT.
    // false is fail-closed while mechanics removes/reinstalls the optional CRServo.
    public static boolean KICKER_SERVO_ENABLED = false;
    public static final double   KICKER_OUT_SPEED = 0.85;
    public static final double KICKER_REVERSE_SPEED = -0.7;
    public static final double KICKER_SERVO_FORWARD_POWER = 0.5;
    public static final double KICKER_SERVO_REVERSE_POWER = -0.5;
    public static final double KICKER_SERVO_STOP_POWER = 0.0;
    public static long KICKER_EXTEND_TIME_MS = 632;
    public static long KICKER_RETRACT_DELAY_MS = 60;

    //Modelo RPM-por-distancia (MP-06/DEC-012)
    public static class ShooterModelConstants {
        // Alineado con el gate T8.1 (±100 RPM sostenidas 250 ms).
        public static double RPM_MODEL_TOLERANCE_RPM = 100.0;
        // Dataset real: TBD-BLOCKING Tuning Paso 5 (handoff-task.md). Hasta
        // entonces los modelos solo se ejercitan con datos sintéticos en tests.
    }

    //Visión (Limelight, MP-03)
    public static class VisionConstants {
        public static double LIMELIGHT_POLL_RATE_HZ = 50.0;
        // Observaciones más viejas que esto se rechazan como STALE (fail-closed).
        public static double LIMELIGHT_MAX_STALENESS_MS = 200.0;
        // TBD-BLOCKING DEC-028: confirmar el índice real al configurar la Limelight.
        public static int LIMELIGHT_PIPELINE_APRILTAG = 0;
    }

    //Torreta
    public static class TurretConstants {
        // 1. Baja el KP: Si estaba en 0.04, intenta con 0.015 o 0.02.
        // Queremos que se mueva más suave.
        public static double TURRET_KP = 0.012;

        // 2. Baja la potencia máxima: Así, aunque el error sea mucho,
        // la torreta no saldrá disparada a máxima velocidad.
        public static double TURRET_MAX_POWER = 0.5;

        // 3. Aumenta la tolerancia: Un margen de 2 o 3 grados ayuda a que
        // no intente corregir movimientos milimétricos que causan vibración.
        public static double TURRET_ERROR_TOLERANCE = 2.5;
        public static long TAG_LOSS_HOLD_MS = 250;
        public static final long TURRET_ARM_HOLD_MS = 1000;
        public static int BLUE_GOAL_TAG_ID = 20;
        public static int RED_GOAL_TAG_ID = 24;
    }
}
