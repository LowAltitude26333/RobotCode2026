package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.LowAltitudeConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

/** DECODE goal tags missing from the FTC SDK 10.3 default game library. */
public final class DecodeGoalTags {
    private static final double TAG_SIZE_INCHES = 6.5;

    private DecodeGoalTags() {
    }

    public static AprilTagLibrary createLibrary() {
        return new AprilTagLibrary.Builder()
                .addTag(LowAltitudeConstants.TurretConstants.BLUE_GOAL_TAG_ID,
                        "BlueTarget", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(LowAltitudeConstants.TurretConstants.RED_GOAL_TAG_ID,
                        "RedTarget", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .build();
    }
}
