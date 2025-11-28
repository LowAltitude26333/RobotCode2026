package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotMap;

public class AngularSubsystem extends SubsystemBase {
    private final Servo one;
    private final Servo two;

    private static final double MAX_DEGREES = 300.0;

    private double currentDegreesOne = 0.0;
    private double currentDegreesTwo = 0.0;

    public AngularSubsystem(HardwareMap hardwareMap) {
        one = hardwareMap.get(Servo.class, RobotMap.SERVO_1);
        two = hardwareMap.get(Servo.class, RobotMap.SERVO_2);

        setServoOnePosition(150);
        setServoTwoPosition(150);
    }


    public void setServoOnePosition(double degrees) {
        currentDegreesOne = degrees;
        one.setPosition(normalize(degrees));
    }

    public void setServoTwoPosition(double degrees) {
        currentDegreesTwo = degrees;
        two.setPosition(normalize(degrees));
    }

    private double normalize(double degrees) {
        return Math.max(0.0, Math.min(1.0, degrees / MAX_DEGREES));
    }


    public void position1() {
        setServoOnePosition(300);
        setServoTwoPosition(0);
    }

    public void position2() {
        setServoOnePosition(150);
        setServoTwoPosition(150);
    }

    public void stop() {
        setServoOnePosition(currentDegreesOne);
        setServoTwoPosition(currentDegreesTwo);
    }
}
