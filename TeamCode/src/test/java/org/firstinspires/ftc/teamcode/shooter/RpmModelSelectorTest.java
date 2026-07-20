package org.firstinspires.ftc.teamcode.shooter;

import static org.firstinspires.ftc.teamcode.shooter.RpmModelsTest.samples;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.junit.Test;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

public class RpmModelSelectorTest {

    private static final List<ShotSample> LINEAR_CALIBRATION = samples(
            new double[]{20, 40, 60, 80}, new double[]{2200, 2600, 3000, 3400});

    @Test
    public void physicalGateSelectsSimplestPassingCandidate() {
        Map<RpmModelKind, List<ShotTrial>> trials = new EnumMap<>(RpmModelKind.class);
        trials.put(RpmModelKind.LINEAR, completeTrials(9, 8));

        RpmModelSelector.Result result = RpmModelSelector.select(LINEAR_CALIBRATION, trials);

        assertTrue(result.metCriteria);
        assertEquals(RpmModelKind.LINEAR, result.kind);
        assertTrue(result.model instanceof LinearRpmModel);
    }

    @Test
    public void candidateResultsAreNotSharedBetweenModels() {
        Map<RpmModelKind, List<ShotTrial>> trials = new EnumMap<>(RpmModelKind.class);
        trials.put(RpmModelKind.LINEAR, completeTrials(8, 8));
        trials.put(RpmModelKind.QUADRATIC,
                completeTrials(RpmModelKind.QUADRATIC, 9, 8));

        RpmModelSelector.Result result = RpmModelSelector.select(LINEAR_CALIBRATION, trials);

        assertTrue(result.metCriteria);
        assertEquals(RpmModelKind.QUADRATIC, result.kind);
        assertTrue(result.model instanceof QuadraticRpmModel);
    }

    @Test
    public void oneSessionCannotCertifyModel() {
        Map<RpmModelKind, List<ShotTrial>> trials = new EnumMap<>(RpmModelKind.class);
        List<ShotTrial> oneSession = new ArrayList<>();
        addGroup(oneSession, RpmModelKind.LINEAR,
                ShotDatasetRole.CALIBRATION, "session-1", "60in", 10);
        addGroup(oneSession, RpmModelKind.LINEAR,
                ShotDatasetRole.HOLDOUT, "session-1", "75in", 10);
        trials.put(RpmModelKind.LINEAR, oneSession);

        RpmModelSelector.Result result = RpmModelSelector.select(LINEAR_CALIBRATION, trials);

        assertFalse(result.metCriteria);
        assertNull(result.kind);
        assertTrue(result.rationale.contains("NO_MODEL_MET_CRITERIA"));
        assertEquals(0.0, result.model.rpmForDistance(60), 0.0);
    }

    @Test
    public void everySessionDistanceGroupMustHaveTenTrials() {
        Map<RpmModelKind, List<ShotTrial>> trials = new EnumMap<>(RpmModelKind.class);
        List<ShotTrial> incomplete = completeTrials(9, 8);
        incomplete.remove(0);
        trials.put(RpmModelKind.LINEAR, incomplete);

        RpmModelSelector.Result result = RpmModelSelector.select(LINEAR_CALIBRATION, trials);

        assertFalse(result.metCriteria);
        assertTrue(result.rationale.contains("requiere >=10"));
    }

    @Test
    public void calibrationAndHoldoutMustCoverTheSameSessions() {
        Map<RpmModelKind, List<ShotTrial>> trials = new EnumMap<>(RpmModelKind.class);
        List<ShotTrial> splitSessions = new ArrayList<>();
        addGroup(splitSessions, RpmModelKind.LINEAR,
                ShotDatasetRole.CALIBRATION, "cal-1", "60in", 9);
        addGroup(splitSessions, RpmModelKind.LINEAR,
                ShotDatasetRole.CALIBRATION, "cal-2", "60in", 9);
        addGroup(splitSessions, RpmModelKind.LINEAR,
                ShotDatasetRole.HOLDOUT, "hold-1", "75in", 8);
        addGroup(splitSessions, RpmModelKind.LINEAR,
                ShotDatasetRole.HOLDOUT, "hold-2", "75in", 8);
        trials.put(RpmModelKind.LINEAR, splitSessions);

        RpmModelSelector.Result result = RpmModelSelector.select(LINEAR_CALIBRATION, trials);

        assertFalse(result.metCriteria);
        assertTrue(result.rationale.contains("missing from one or more sessions"));
    }

    @Test
    public void calibrationNeedsNineOfTenAndHoldoutEightOfTen() {
        Map<RpmModelKind, List<ShotTrial>> trials = new EnumMap<>(RpmModelKind.class);
        trials.put(RpmModelKind.LINEAR, completeTrials(8, 8));
        assertFalse(RpmModelSelector.select(LINEAR_CALIBRATION, trials).metCriteria);

        trials.put(RpmModelKind.LINEAR, completeTrials(9, 7));
        assertFalse(RpmModelSelector.select(LINEAR_CALIBRATION, trials).metCriteria);

        trials.put(RpmModelKind.LINEAR, completeTrials(9, 8));
        assertTrue(RpmModelSelector.select(LINEAR_CALIBRATION, trials).metCriteria);
    }

    @Test
    public void trialsMustUseCandidateAndMeetRpmReadinessGate() {
        Map<RpmModelKind, List<ShotTrial>> trials = new EnumMap<>(RpmModelKind.class);
        List<ShotTrial> wrongTarget = completeTrials(9, 8);
        ShotTrial original = wrongTarget.get(0);
        wrongTarget.set(0, new ShotTrial(original.sessionId, original.distanceGroupId,
                original.modelKind, original.role, original.distanceInches, original.targetRpm + 2.0,
                original.measuredRpmAtFeed, original.rpmReadyHoldMs, original.outcome));
        trials.put(RpmModelKind.LINEAR, wrongTarget);
        assertFalse(RpmModelSelector.select(LINEAR_CALIBRATION, trials).metCriteria);

        List<ShotTrial> offSpeed = completeTrials(9, 8);
        original = offSpeed.get(0);
        offSpeed.set(0, new ShotTrial(original.sessionId, original.distanceGroupId,
                original.modelKind, original.role, original.distanceInches, original.targetRpm,
                original.targetRpm + 101.0, 250, original.outcome));
        trials.put(RpmModelKind.LINEAR, offSpeed);
        assertFalse(RpmModelSelector.select(LINEAR_CALIBRATION, trials).metCriteria);

        List<ShotTrial> notReady = completeTrials(9, 8);
        original = notReady.get(0);
        notReady.set(0, new ShotTrial(original.sessionId, original.distanceGroupId,
                original.modelKind, original.role, original.distanceInches, original.targetRpm,
                original.targetRpm, 249, original.outcome));
        trials.put(RpmModelKind.LINEAR, notReady);
        assertFalse(RpmModelSelector.select(LINEAR_CALIBRATION, trials).metCriteria);
    }

    @Test
    public void invalidCalibrationIsRejected() {
        try {
            RpmModelSelector.select(null,
                    new EnumMap<RpmModelKind, List<ShotTrial>>(RpmModelKind.class));
            fail("null calibration should fail");
        } catch (IllegalArgumentException expected) {
            // Fit data is a required input even though it cannot certify a model by itself.
        }
    }

    private static List<ShotTrial> completeTrials(int calibrationHits, int holdoutHits) {
        return completeTrials(RpmModelKind.LINEAR, calibrationHits, holdoutHits);
    }

    private static List<ShotTrial> completeTrials(RpmModelKind modelKind,
                                                  int calibrationHits, int holdoutHits) {
        List<ShotTrial> trials = new ArrayList<>();
        addGroup(trials, modelKind,
                ShotDatasetRole.CALIBRATION, "session-1", "60in", calibrationHits);
        addGroup(trials, modelKind,
                ShotDatasetRole.CALIBRATION, "session-2", "60in", calibrationHits);
        addGroup(trials, modelKind,
                ShotDatasetRole.HOLDOUT, "session-1", "75in", holdoutHits);
        addGroup(trials, modelKind,
                ShotDatasetRole.HOLDOUT, "session-2", "75in", holdoutHits);
        return trials;
    }

    private static void addGroup(List<ShotTrial> trials, RpmModelKind modelKind,
                                 ShotDatasetRole role,
                                 String sessionId, String distanceGroupId, int hits) {
        for (int i = 0; i < 10; i++) {
            double distance = role == ShotDatasetRole.CALIBRATION ? 60.0 : 75.0;
            double targetRpm = 20.0 * distance + 1800.0;
            trials.add(new ShotTrial(sessionId, distanceGroupId, modelKind, role,
                    distance, targetRpm, targetRpm - 10.0, 250,
                    i < hits ? ShotOutcome.SCORED : ShotOutcome.SHORT));
        }
    }
}
