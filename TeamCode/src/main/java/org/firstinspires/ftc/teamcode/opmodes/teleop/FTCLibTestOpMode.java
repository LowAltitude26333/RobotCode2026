package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.hardware.motors.Motor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "FTCLib Test", group = "Test")
public class FTCLibTestOpMode extends com.arcrobotics.ftclib.command.CommandOpMode {

    private Motor leftMotor;

    @Override
    public void initialize() {
        leftMotor = new Motor(hardwareMap, "left_drive");
    }

    @Override
    public void run() {
        double power = -gamepad1.left_stick_y;
        leftMotor.set(power);
    }
}
