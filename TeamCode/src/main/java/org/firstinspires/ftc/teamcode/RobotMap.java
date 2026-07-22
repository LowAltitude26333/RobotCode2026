package org.firstinspires.ftc.teamcode;

public final class RobotMap {

    private RobotMap() {
    }

    public static final String FRONT_LEFT_MOTOR = "leftFront";
    public static final boolean FRONT_LEFT_MOTOR_IS_INVERTED = true;

    public static final String FRONT_RIGHT_MOTOR = "rightFront";//0
    public static final boolean FRONT_RIGHT_MOTOR_IS_INVERTED = false;

    public static final String BACK_LEFT_MOTOR = "leftBack";
    public static final boolean BACK_LEFT_MOTOR_IS_INVERTED = true;

    public static final String BACK_RIGHT_MOTOR = "rightBack";//1
    public static final boolean BACK_RIGHT_MOTOR_IS_INVERTED = true;

    public static final String IMU = "imu";

    // The dead-wheel encoders are wired to these drivetrain motor encoder inputs.
    public static final String ODOMETRY_PARALLEL_0 = FRONT_RIGHT_MOTOR;
    public static final String ODOMETRY_PARALLEL_1 = FRONT_LEFT_MOTOR;
    public static final String ODOMETRY_PERPENDICULAR = BACK_RIGHT_MOTOR;

    //Shooter
    public static final String SHOOTER_MOTOR = "Shooter";
    public static final boolean SHOOTER_MOTOR_IS_INVERTED = true;
    public static final boolean SHOOTER_ENCODER_IS_INVERTED = false;

    public static final String INTAKE_MOTOR = "intakeMotor";
    public static final boolean INTAKE_MOTOR_IS_INVERTED = false;

    public static final String KICKER_MOTOR = "kickerMotor";
    public static final boolean KICKER_MOTOR_IS_INVERTED = false;
    public static final String KICKER_SERVO = "kickerServo";
    public static final boolean KICKER_SERVO_IS_INVERTED = false;

    //Torreta
    public static final String TURRET_MOTOR = "torretaMotor";
    public static final boolean TURRET_MOTOR_IS_INVERTED = true;

    // TBD-BLOCKING DEC-028: nombre pendiente de confirmar cuando eléctrica
    // configure la Limelight en el Driver Station (mapping/red/pipeline).
    public static final String LIMELIGHT = "limelight";
}
