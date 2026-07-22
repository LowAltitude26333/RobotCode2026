package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RobotMap;


/*
=======
Uso de "Panels" para el tuning. Ir a https://panels.bylazar.com/docs/com.bylazar.docs/Accessing%20Panels/ para info.

Evaluar el uso de Predictive Braking.

Evaluar el uso de un doble PID

=======
 */
public class Constants {
    public static final double ROBOT_MASS_KG = 8.5;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(ROBOT_MASS_KG)
            //.headingPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))   //Tuning de Heading checar Tuning -> Heading
            .useSecondaryTranslationalPIDF(true)    //habilitar el uso de PID doble. Se puede poner en falso
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true);

            //.forwardZeroPowerAcceleration(deceleration)       //tuning Zero Power Acceleration.
            //.lateralZeroPowerAcceleration(deceleration)       //checar Tuning -> Driving Algorithm -> PIDF'S -> Zero Power Acceleration

            //.translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
            //.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0.0,0.01,0.6,0.0)) //main PID drive
            //.secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0,0.01,0.6,0.01)) // secondary PID drive
            //.centripetalScaling(0.005)     //ver el video de Tuning -> Drive Algorithm -> PIDF's -> Centripedal para más info

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(RobotMap.FRONT_RIGHT_MOTOR)
            .rightRearMotorName(RobotMap.BACK_RIGHT_MOTOR)
            .leftRearMotorName(RobotMap.BACK_LEFT_MOTOR)
            .leftFrontMotorName(RobotMap.FRONT_LEFT_MOTOR)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

            /*
            .xVelocity(velocity)        //tuning para velocidad de autos
            .yVelocity(velocity)        //checar Tuning -> Velocity Tuners
             */

    //braking start = que tan temprano empieza a frenar más bajo es más tarde, más alto más temprano. Braking Strenght que tan fuerte frena, más alto más fuerte, más bajo más suave.
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    // ===== TBD-BLOCKING: calibración de pods (Tuning Paso 2, plan-paralelo-20h.md secc. 2) =====
    // Los tres pods se leen por los puertos de encoder del Hub — NO existe un
    // Pinpoint físico (plan-paralelo-20h.md secc. 1). Mapeo coherente con Road
    // Runner (ThreeDeadWheelLocalizer): par0=rightFront, par1=leftFront, perp=rightBack.
    // Valores placeholder NO calibrados. leftPodY y rightPodY no pueden ser iguales:
    // la matemática three-wheel divide entre (leftPodY - rightPodY) y con offsets
    // idénticos produce NaN si alguien corre Tuning.java antes de calibrar.
    // Tuning Paso 2 escribe aquí los valores medidos y voltea el flag a true.
    public static final boolean LOCALIZER_OFFSETS_CALIBRATED = true;
    public static final double TBD_FORWARD_TICKS_TO_INCHES = 0.00191837052073273;
    public static final double TBD_STRAFE_TICKS_TO_INCHES = .001989436789;
    // AngularRampLogger de Road Runner midió automáticamente los offsets efectivos
    // de estos mismos tres pods. Se traducen a las convenciones de Pedro:
    // par1=left (+Y), par0=right (-Y), perp=strafe (-X). El scalar de giro usa
    // la escala del par de encoders paralelos; no la escala empírica de strafe.
    public static final double TBD_TURN_TICKS_TO_INCHES = TBD_FORWARD_TICKS_TO_INCHES;
    public static final double TBD_LEFT_POD_Y_INCHES = 6.942896027755779;
    public static final double TBD_RIGHT_POD_Y_INCHES = -6.089060628403165;
    public static final double TBD_STRAFE_POD_X_INCHES = -4.636066386085883;
    // Pedro modela la dirección como double: Encoder.FORWARD=1, Encoder.REVERSE=-1.
    public static final double TBD_LEFT_ENCODER_DIRECTION = Encoder.REVERSE;
    public static final double TBD_RIGHT_ENCODER_DIRECTION = Encoder.REVERSE;
    public static final double TBD_STRAFE_ENCODER_DIRECTION = Encoder.REVERSE;

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(TBD_FORWARD_TICKS_TO_INCHES)
            .strafeTicksToInches(TBD_STRAFE_TICKS_TO_INCHES)
            .turnTicksToInches(TBD_TURN_TICKS_TO_INCHES)
            .leftEncoder_HardwareMapName(RobotMap.ODOMETRY_PARALLEL_1)      // leftFront (par1, pod izquierdo)
            .rightEncoder_HardwareMapName(RobotMap.ODOMETRY_PARALLEL_0)     // rightFront (par0, pod derecho)
            .strafeEncoder_HardwareMapName(RobotMap.ODOMETRY_PERPENDICULAR) // rightBack (perp)
            .leftEncoderDirection(TBD_LEFT_ENCODER_DIRECTION)
            .rightEncoderDirection(TBD_RIGHT_ENCODER_DIRECTION)
            .strafeEncoderDirection(TBD_STRAFE_ENCODER_DIRECTION)
            .leftPodY(TBD_LEFT_POD_Y_INCHES)
            .rightPodY(TBD_RIGHT_POD_Y_INCHES)
            .strafePodX(TBD_STRAFE_POD_X_INCHES);


    public static Follower createFollower(HardwareMap hardwareMap) {
        if (!LOCALIZER_OFFSETS_CALIBRATED) {
            // Fail-loudly: el scaffold viejo crasheaba en init ("rightRear"
            // inexistente); reparado ya construye, así que esta advertencia en
            // Driver Station evita usar Pedro con geometría placeholder sin saberlo.
            RobotLog.addGlobalWarningMessage(
                    "Pedro: offsets de pods SIN CALIBRAR (Tuning Paso 2 pendiente); "
                            + "la pose sera geometricamente incorrecta");
        }
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}
