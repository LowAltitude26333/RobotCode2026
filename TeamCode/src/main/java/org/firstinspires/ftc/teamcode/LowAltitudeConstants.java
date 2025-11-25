package org.firstinspires.ftc.teamcode;

public class LowAltitudeConstants {

    public static final double INTAKE_IN_SPEED = 0.0;

    //Chasis
    public static final double CHASSIS_POWER = 0.8;

    //Shooter
    public static final double SHOOTER_ON_SPEED = 0.5;
    public static final double SHOOTER_OFF_SPEED = 0.0;
    public static final double SHOOTER_INTAKE_SPEED = -0.5;

    //PID Shooter
    public static final double SHOOTER_KP = 0.37;
    public static final double SHOOTER_KI = 0.05;
    public static final double SHOOTER_KD = 1.02;
    public static final double SHOOTER_KF = 0.7;

    //Tolerance
    public static final double RPM_OFFSET = 50;
    public static final double TICKS_PER_REV = 28.0;
    public static final double GEAR_RATIO = 2;


    //Feedforward Shooter
    public static final double SHOOTER_KS = 0.37;
    public static final double SHOOTER_KV = 0.05;

    public static final double INTAKE_STOP = 0.0;
    public static final double INTAKE_RETURN = 0.0;

    // --- SHOOTER HOOD CONSTANTS ---
    public static final double HOOD_MIN_ANGLE_DEG = 0;
    public static final double HOOD_MAX_ANGLE_DEG = 300; // Revisa specs del servo (goBILDA vs REV)
    public static final double HOOD_MIN_LIMIT = 25; // Límite de seguridad en código
    public static final double HOOD_MAX_LIMIT = 55; // Límite de seguridad en código

    public enum HoodPosition {
        WALL_SHOT(25.0),    // Tiro cercano
        MID_FIELD(35.0),    // Tiro medio
        LONG_SHOT(50.0);    // Tiro lejano

        public final double angle;

        HoodPosition(double angle) {
            this.angle = angle;
        }
    }

    // --- KICKER CONSTANTS ---
    public static final double KICKER_SPEED = 1.0;
    public static final int KICKER_CYCLE_TICKS = 288; // Solo si usas encoder (ej. motor Core Hex)
}
