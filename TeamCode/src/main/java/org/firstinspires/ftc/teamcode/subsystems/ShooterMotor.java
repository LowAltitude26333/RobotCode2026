package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.RobotSafety;

public class ShooterMotor extends SubsystemBase {
    private final MotorEx motorLeader;


    public ShooterMotor(HardwareMap hardwareMap) {


        motorLeader = new MotorEx(hardwareMap, RobotMap.SHOOTER_MOTOR);
        RobotSafety.registerShutdown(this::stop);


    }
    public void right2() {

        motorLeader.set(0.9);


    }

    public void left2() {

        motorLeader.set((-0.9));


    }

    public void stop() {

        motorLeader.set((0));


    }
}
