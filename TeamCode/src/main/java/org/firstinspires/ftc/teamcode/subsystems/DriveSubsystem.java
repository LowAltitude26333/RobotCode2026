package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotMap;


public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive mecanumDrive;

    private final MotorEx frontLeft, frontRight, backLeft, backRight;



    public DriveSubsystem(HardwareMap hardwareMap) {
        frontLeft = new MotorEx(hardwareMap, RobotMap.FRONT_LEFT_MOTOR);
        frontRight = new MotorEx(hardwareMap, RobotMap.FRONT_RIGHT_MOTOR);
        backLeft = new MotorEx(hardwareMap, RobotMap.BACK_LEFT_MOTOR);
        backRight = new MotorEx(hardwareMap, RobotMap.BACK_RIGHT_MOTOR);

        backLeft.setInverted(RobotMap.BACK_LEFT_MOTOR_IS_INVERTED);
        frontLeft.setInverted(RobotMap.FRONT_LEFT_MOTOR_IS_INVERTED);
        backRight.setInverted(RobotMap.BACK_RIGHT_MOTOR_IS_INVERTED);
        frontRight.setInverted(RobotMap.FRONT_RIGHT_MOTOR_IS_INVERTED);

        frontLeft.encoder.reset();
        frontRight.encoder.reset();
        backLeft.encoder.reset();
        backRight.encoder.reset();

        mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);


    }



    public void drive(double strafe, double forward, double turn) {
        mecanumDrive.driveRobotCentric(-strafe, forward, -turn);
    }




}