package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name="Test Color", group="Sensor")
public class TestColor extends LinearOpMode {
    NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");

        waitForStart();

        while (opModeIsActive()) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);
            telemetry.addData("Alpha", "%.3f", colors.alpha);
            telemetry.update();
        }
    }
}