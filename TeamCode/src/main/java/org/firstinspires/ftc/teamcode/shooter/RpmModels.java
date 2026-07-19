package org.firstinspires.ftc.teamcode.shooter;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

/** Utilidades internas compartidas por los modelos RPM. */
final class RpmModels {

    private RpmModels() {
    }

    static int distinctDistanceCount(List<ShotSample> samples) {
        if (samples == null) {
            return 0;
        }
        Set<Double> distances = new HashSet<>();
        for (ShotSample s : samples) {
            distances.add(s.distanceInches);
        }
        return distances.size();
    }
}
