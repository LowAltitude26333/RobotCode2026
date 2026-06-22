package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.RobotSafety;

public class ShooterMotor extends SubsystemBase {
    private final MotorEx motorLeader;


    public ShooterMotor(HardwareMap hardwareMap, Telemetry telemetry) {


        motorLeader = new MotorEx(hardwareMap, RobotMap.SHOOTER_MOTOR_1);
        RobotSafety.registerShutdown(this::stop);


    }
    public void right2() {

        motorLeader.set(0.5);


    }

    public void left2() {

        motorLeader.set((-0.5));


    }

    public void stop() {

        motorLeader.set((0));


    }
}
