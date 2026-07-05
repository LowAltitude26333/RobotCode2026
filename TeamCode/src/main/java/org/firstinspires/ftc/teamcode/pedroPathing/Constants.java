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


/*
=======
Uso de "Panels" para el tuning. Ir a https://panels.bylazar.com/docs/com.bylazar.docs/Accessing%20Panels/ para info.

Evaluar el uso de Predictive Braking.
=======
 */
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5) //Masa en kilogramos (falta poner)
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
            .rightFrontMotorName("rightFront")   //nombres de los motores. Si no funcionan los moteores probablemente esten mal los mombres.
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

            /*
            .xVelocity(velocity)        //tuning para velocidad de autos
            .yVelocity(velocity)        //checar Tuning -> Velocity Tuners
             */

    //braking start = que tan temprano empieza a frenar más bajo es más tarde, más alto más temprano. Braking Strenght que tan fuerte frena, más alto más fuerte, más bajo más suave.
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(.001989436789)
            .strafeTicksToInches(.001989436789)
            .turnTicksToInches(.001989436789)
            .leftPodY(1)
            .rightPodY(-1)
            .strafePodX(-2.5)
            .leftEncoder_HardwareMapName("leftFront")       //poner bien los nombres de las pinpoint
            .rightEncoder_HardwareMapName("rightRear")
            .strafeEncoder_HardwareMapName("rightFront")
            .leftEncoderDirection(Encoder.FORWARD)      //cambiar a REVERSE si está al revés
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .leftPodY(0)            //offsets tienen que medirse en inches
            .rightPodY(0)
            .strafePodX(0);
            /*
            .forwardTicksToInches(multiplier)                //multiplicadores de auto. Requiere tuning
            .strafeTicksToInches(multiplier)                 //checar Tuning -> Localization -> Three Wheel
            .turnTicksToInches(multiplier)
             */


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}