package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotMap;

public class ShooterMotor extends SubsystemBase {
    private final MotorEx motorLeader;
    private final MotorEx motorFollower;

    public ShooterMotor(HardwareMap hardwareMap, Telemetry telemetry) {


        motorLeader = new MotorEx(hardwareMap, RobotMap.SHOOTER_MOTOR_1);
        motorFollower = new MotorEx(hardwareMap, RobotMap.SHOOTER_MOTOR_2);

    }

    public void right2() {

        motorLeader.set(0.9);
        motorFollower.set(0.9);

    }

    public void left2() {

        motorLeader.set((-0.9));
        motorFollower.set(-0.9);

    }

    public void stop() {

        motorLeader.set((0));
        motorFollower.set(0);

    }
}
