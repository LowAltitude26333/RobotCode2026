package org.firstinspires.ftc.teamcode;

public class LowAltitudeConstants {


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

    public static final double INTAKE_IN_SPEED = 0.7;
    public static final double INTAKE_STOP = 0.0;
    public static final double INTAKE_RETURN = -0.7;
}
