# FTC Robot Development Instructions

This is a FIRST Tech Challenge Robot Controller project. AI assistance supports
students and mentors; it does not replace understanding the robot, reviewing the
code, or validating behavior on hardware.

## Canonical Workspace

- The only authoritative working copy for this project is
  `C:\dev\RobotCode2026`.
- Before reading, editing, building, hashing, or producing an APK, verify that the
  repository root resolves exactly to `C:\dev\RobotCode2026`. If it does not,
  stop and switch to the canonical workspace before continuing.
- Do not edit, build, deploy, or use artifacts from
  `C:\Users\brito\OneDrive\Documentos\FTC\RobotCode2026`; that copy is
  non-authoritative and may fail to upload or build correctly.
- APK paths, hashes, test reports, handoffs, and commissioning evidence must come
  from the canonical workspace. Preserve unrelated user changes already present
  there.

## Instruction Priority

- Follow system, developer, and user instructions first. Among repository
  instruction files, the closest `AGENTS.md` that applies to a file wins.
- Treat every inventory in this file as orientation. Reinspect the selected source
  branch before each task because OpModes, mappings, and active implementations can
  change.
- When repository evidence conflicts with this snapshot, report the mismatch and
  use the selected branch as the source of truth.

## Branch And Source Layout

Snapshot date: 2026-06-22.

- `main` contains the complete robot source restored from
  `backup/main-before-split-20260619`, the critical-findings remediation, this
  `AGENTS.md`, and repository documentation under `docs/`.
- `backup/main-before-split-20260619` at `e42e7cf` is the frozen complete source
  baseline used for that restoration. The architecture inventory below describes
  that baseline unless a newer `main` implementation is explicitly noted.
- `temporada` at `9942d44` is an older complete robot branch. It predates the turret,
  replacement autonomous OpModes, intake test OpMode, and the newest shooter and
  hardware-map changes.
- `alpha`, `drivetrain`, `intake`, `kicker`, `shooter`, and `shooterHood` are feature
  or historical refs. Do not assume a branch is current from its name.
- Ignored `build/` trees may remain in a checkout. Generated
  classes and APK/AAR outputs are not editable source and must not be used to infer
  the selected branch's current behavior.

Before changing robot code, confirm the intended base and inspect its current tree.
Use an appropriate branch or worktree, preserve unrelated changes, and never
switch/reset a dirty worktree destructively.

Useful branch-aware inspection commands from the repository root:

```powershell
git status --short
git branch -a -vv
git log --all --graph --decorate --oneline --max-count=80

$sourceRef = "backup/main-before-split-20260619"
git ls-tree -r --name-only $sourceRef TeamCode/src/main/java/org/firstinspires/ftc/teamcode
git show "${sourceRef}:settings.gradle"
git show "${sourceRef}:TeamCode/build.gradle"
git grep -n -E '@(com\.qualcomm\.robotcore\.eventloop\.opmode\.)?(TeleOp|Autonomous|Disabled)' $sourceRef -- TeamCode/src/main/java
git grep -n -E 'hardwareMap|get\(|setDirection|setInverted|setMode|setPosition|setPower|set\(' $sourceRef -- TeamCode/src/main/java/org/firstinspires/ftc/teamcode
```

When a complete source branch is checked out, prefer direct `rg` inspection:

```powershell
rg --files TeamCode/src/main/java/org/firstinspires/ftc/teamcode
rg -n --glob '*.java' '@(com\.qualcomm\.robotcore\.eventloop\.opmode\.)?(TeleOp|Autonomous|Disabled)' TeamCode/src/main/java
rg -n --glob '*.java' 'extends SubsystemBase|extends CommandBase|extends SequentialCommandGroup' TeamCode/src/main/java
rg -n --glob '*.java' 'hardwareMap|get\(|setDirection|setInverted|setMode|setPosition|setPower|set\(' TeamCode/src/main/java/org/firstinspires/ftc/teamcode
```

## Repository Boundaries

- Robot code belongs primarily in
  `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
- Make robot-code changes inside `TeamCode` unless the user explicitly requests a
  different module.
- `FtcRobotController` contains the FTC SDK/controller application. Do not modify it
  unless explicitly requested.
- `MeepMeepTesting` is desktop Road Runner trajectory simulation. Simulation there
  does not verify behavior on the physical robot.
- Do not update Gradle, the Android Gradle Plugin, Java, Android dependencies, the
  FTC SDK, or accept automatic Android Studio upgrades unless explicitly requested.
- Do not add a dependency without explaining why existing SDK and libraries are
  insufficient.
- Preserve repository structure and avoid unrelated refactors.
- Preserve user changes in a dirty worktree. Never reset, overwrite, or revert work
  unrelated to the request.

## Required Repository Discovery

Before proposing or making code changes:

1. Read every applicable `AGENTS.md`; inspect `git status --short`, the current
   branch, and its tracked tree.
2. Confirm that the selected ref contains `settings.gradle`, `TeamCode/build.gradle`,
   and source files. Read them and the relevant package tree.
3. Inspect the complete call path: registered OpMode, `RobotContainer` or direct
   composition, control profile, command, subsystem, drive/localizer, constants, and
   `RobotMap` as applicable.
4. Search every affected `hardwareMap` lookup, direction/inversion, encoder mode,
   zero-power behavior, power/position limit, timeout, command requirement, and stop
   path.
5. Identify duplicate or competing implementations and prove which one the selected
   OpMode instantiates. Do not infer activity from filenames.
6. State missing hardware facts and provide a short implementation plan with units,
   limits, ownership, stop behavior, and acceptance criteria.

## Project Architecture Snapshot

This snapshot describes `backup/main-before-split-20260619` at `e42e7cf`, not a
physical robot configuration.

- The project is Java with FTCLib command-based code, Road Runner 1.0, FTC Dashboard,
  and MeepMeep.
- `TeamCode/build.gradle` explicitly declares FTC SDK `10.3.0`, FTCLib `2.1.1`, Road
  Runner core/actions `1.0.1`, and Road Runner FTC `0.1.23`. The repository base and
  README came from the SDK 11.0 DECODE project, so verify APIs against declared
  dependencies rather than the README or Git ancestry.
- Dashboard is declared twice (`0.4.16` and `0.4.17`). Do not normalize dependency
  versions as incidental cleanup.
- `RobotContainer` owns drive, intake, PID/feedforward shooter, hood, and kicker. It
  assigns a field-centric drive command and applies `SkywalkerProfile` bindings.
- `MainTeleOp` separately owns the turret and AprilTag processor, then overrides the
  container's drive default command with a direct `RunCommand`. Inspect that override
  before modifying field-centric drive or precision behavior.
- `RobotContainer.initAprilTag()` is currently not called. Its dormant camera name is
  `Webcam`; active turret vision in `MainTeleOp` and `TeleopTorreta` uses `Webcam 1`.
- `DriveSubsystem` owns `MecanumDrive`. `MecanumDrive` actively instantiates
  `TwoDeadWheelLocalizer`; Pinpoint, OTOS, three-dead-wheel, drive-encoder, and tank
  alternatives exist but are not active through that path.
- The active two-dead-wheel localizer reads encoder ports `rightFront` and
  `rightBack`, reverses the perpendicular encoder, and uses the `imu` device.

### Current OpModes

Registered TeleOps in the merged snapshot:

- `MainTeleOp` - `Skywalker TeleOp (Manual Principal)`
- `MainTeleOp2` - `Skywalker TeleOp 2`
- `TeleopTorreta` - `Skywalker Turret: SENTINEL (Solo Conectados)`
- `IntakeTeleOp` - `TeleOp Intake Control`
- `TeleOpApril` - `Skywalker TeleOp w/ April`
- `TeleOpPositionBEta` - `TeleOpBETA 1 ROJO`
- `TeleOpAlignWithPoint` - `boton`
- `TeleOpMotoresPrueba` - `Motores`
- `TeleOpShooter` - `ShooterTeleOpAdaptado` (under `commands/drivetrain`)
- `SystemCheckOpMode` - `SYSTEM CHECK and TUNING`
- `ShooterTuningOpMode` - `Tuning: Shooter & Systems (Manual)`
- `TestColor` - `Test Color`
- `FTCLibTestOpMode` - `FTCLib Test`
- `TeleOpFieldCentric` - `TeleOpFieldCentric`, currently `@Disabled`

Registered autonomous OpModes:

- `AutonomoBetaPosition` - `Beta`
- `AutonomoOfficialBlue` - `AutonomoOfficialBlue`
- `AutonomoOfficialRed2` - `AutonomoOfficialRed`
- `FullOfficialBlue2` - `FullOfficialBlue`
- `FullOfficialRed2` - `FullOfficialRed`
- `AutonomoOfficialRed` - `nada 1`
- `FullOficialBlue` - `nada 1`
- `FullOficialRed` - `nada1`
- `TestShootBurstAuto` - `TEST: Shoot Burst Command`

`AutonomoOfficialRed` and `FullOficialBlue` intentionally reflect the same observed
Driver Station display name, `nada 1`. Treat that collision and the similarly named
old/new autonomous classes as an ambiguity to resolve, not an invitation to rename
or delete an OpMode without approval.

`TuningOpModes` dynamically registers Road Runner tuning routines, including
`LocalizationTest`, `ManualFeedbackTuner`, and `SplineTest`; annotations are not the
only registration mechanism.

### Current Subsystems And Commands

- `DriveSubsystem`: Road Runner mecanum drive, direct/field-centric drive paths, pose
  updates, precision scaling, and action integration.
- `ShooterSubsystem`: one active motor (`Shooter`) with encoder feedback,
  PID/feedforward, voltage compensation, RPM filtering, bang-bang recovery, and
  output clamping. The older dual-motor description no longer matches this snapshot.
- `ShooterMotor`: separate manual one-motor shooter used by shooter/turret test
  OpModes; it is not the PID shooter.
- `ShooterHoodSubsystem`: paired servos with inversion and angle clamping.
- `KickerSubsystem`: raw-power kicker with brake behavior.
- `IntakeSubsystem`: intake forward, reverse, stop, and Road Runner action adapters.
- `TurretSubsystem`: one encoder motor, brake behavior, encoder reset on construction,
  and software travel gates at `-200` and `+200` ticks.
- `ColorSubsystem`: REV color sensor classification and telemetry.
- `TurretFollowTagCommand`: first-detection AprilTag bearing control with proportional
  tracking, clamped power, automatic search sweep, a turret requirement, and stop in
  `end()`.
- Other commands cover drive, precision mode, AprilTag approach, pose actions,
  shooter sequences, burst shooting, color detection, and turning. `PoseStorage`
  carries pose and alliance/precision state between modes.
- `SkywalkerProfile` and `SkywalkerProfileMotorTest` define driver bindings. Confirm
  which profile is instantiated before changing a binding.

### Hardware Names Observed In Code

Exact names in the merged `RobotMap` are:

- Drive: `leftFront`, `rightFront`, `leftBack`, `rightBack`
- Shooter: `Shooter`; `Shooter2` remains declared but is not mapped by the current
  one-motor `ShooterSubsystem` or `ShooterMotor`
- Intake: `intakeMotor`
- Hood: `hoodLeft`, `hoodRight`
- Kicker: `kickerM otor` (contains an embedded space exactly as committed)
- Turret: `torretaMotor`

Active turret vision directly maps `Webcam 1`. The active mecanum path directly maps
`imu`. Other direct or alternative lookups include `Webcam`, `color`, `left_drive`,
`left`, `right`, `par0`, `par1`, `perp`, `pinpoint`, and `sensor_otos`; some belong to
test code, dormant camera setup, tank drive, or inactive localizers.

Never silently correct `kickerM otor`, substitute `Webcam` for `Webcam 1`, or restore
`Shooter2`. Confirm exact names and the intended motor count against the active Robot
Controller configuration first.

## Merge-Specific Hardware Assumptions

The new merged behavior still requires physical confirmation:

- Turret startup assumes the mechanism is physically centered because construction
  resets the encoder to zero. The `-200`/`+200` tick gates are committed constants,
  not verified physical limits.
- Confirm turret motor direction, encoder sign, ticks over the allowed travel,
  hard-stop clearance, cable routing, and whether automatic search at power `0.3`
  is acceptable. Tracking power is capped at `0.5`.
- AprilTag tracking selects the first returned detection and uses its bearing. No tag
  ID filter, camera pose, loss timeout, or explicit VisionPortal close path is present
  in the merged turret OpModes.
- Confirm `Webcam 1`, camera orientation, exposure, tag family/library, and intended
  target IDs on the Robot Controller.
- Confirm that the physical shooter now uses one motor. `Shooter2` remains declared,
  which is evidence of competing hardware assumptions.
- Confirm shooter ticks/revolution, maximum RPM, external ratio, inversion, and
  voltage/feedforward constants before tuning or enabling shooting.
- Confirm hood servo ranges and linkage limits; Dashboard-tunable values and software
  angle clamps do not prove mechanical safety.
- Confirm the exact kicker mapping because `kickerM otor` may be intentional or a
  typo. Do not guess.

## AI-Assisted Development And Student Ownership

- Convert requests into explicit behavior, units, limits, ownership, failure modes,
  stop behavior, and acceptance criteria before implementation.
- A student or responsible reviewer must be able to explain each nontrivial change,
  its data flow, units, failure modes, and actuator shutdown behavior.
- Prefer small diffs that follow established project patterns. Do not replace the
  architecture merely because another pattern is more familiar.
- Verify SDK/vendor APIs against the versions declared by the selected source branch
  and primary documentation.
- Never use AI output as evidence for hardware specifications, game rules, wiring,
  dimensions, or device configuration.
- Never deploy opaque generated code directly to an enabled robot. Compilation,
  simulation, and log review do not replace staged hardware validation.
- Final responses must explain important logic and tradeoffs so the team can own,
  debug, and maintain the result.

## Hardware Safety

- Never invent hardware names or silently substitute a similar device.
- Ask for the Robot Controller configuration when required mappings cannot be
  verified.
- Do not assume motor directions, encoder signs/resolution, gear ratios, pulley or
  wheel dimensions, servo travel, current limits, IMU orientation, odometry geometry,
  camera pose, turret center, or mechanism dimensions.
- Never remove or weaken limits, timeouts, current limits, interlocks, watchdogs, or
  emergency-stop behavior.
- Every motorized mechanism needs an explicit zero-power path on command interruption
  and OpMode stop. Verify FTCLib requirements and default-command behavior.
- Preserve software ranges consistent with verified physical limits. Raw servo
  positions and Dashboard-tunable limits require special scrutiny.
- Avoid unbounded loops, blocking command execution, and sleeps that prevent
  scheduler updates, stop requests, or actuator shutdown. Existing blocking calls
  are not precedent for adding more.
- Keep the test area clear, restrain or raise the robot when appropriate, announce
  enabling, retain immediate Stop/E-stop access, and begin at reduced power.
- Validate in stages: static review, compilation, simulation where meaningful,
  robot-disabled inspection, restrained/on-blocks testing, then controlled low-power
  field testing.
- Clearly label behavior that still needs physical validation. Never claim physical
  safety or correctness from compilation or simulation alone.

## Implementation Rules

- Follow existing FTCLib command/subsystem ownership and Road Runner APIs.
- Keep hardware ownership in the established subsystem or drive class. Do not map the
  same actuator independently in multiple active components.
- Commands must declare requirements and clean up actuators in interruption/end paths.
- Do not add blocking loops to command-based OpModes or commands.
- Do not change the active localizer, drive type, axes, motor directions, tuning
  constants, coordinate conventions, autonomous display names, or hardware mappings
  as incidental cleanup.
- Do not enable disabled, test, system-check, or tuning OpModes unless explicitly
  requested.
- Keep units explicit in names/comments where ambiguity can affect hardware.
- Preserve safety clamps in ordinary methods and Road Runner `Action` adapters; every
  alternate control path must enforce the same verified limits.
- Turret commands must retain the subsystem requirement, encoder travel checks, power
  clamp, and interruption stop unless a reviewed replacement provides equivalent or
  stronger protection.

## Verification Workflow

After modifying robot code on a complete source branch:

1. Review the full diff and changed files for accidental scope expansion.
2. Recheck hardware names, units, bounds, directions, encoder modes, command
   requirements, interruption behavior, VisionPortal lifecycle, and OpMode stop paths.
3. Run focused static checks/tests and Road Runner or MeepMeep simulation when they
   exercise the changed behavior.
4. From Windows PowerShell at the complete project root, run:

   ```powershell
   .\gradlew.bat assembleDebug
   ```

5. Fix compilation failures caused by the change. Report unrelated/environmental
   failures without modifying build tooling unless requested.
6. Summarize files changed, behavior, build/test results, assumptions, and an explicit
   physical validation checklist.

For documentation-only changes, also run `git diff --check` and review the final
diff. Robot-code changes on `main` must complete the build workflow above.

## Primary References

- [FTC Docs: OpMode structure](https://ftc-docs.firstinspires.org/en/latest/programming_resources/android_studio_java/opmode/opmode.html)
- [FTC Docs: hardware configuration](https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html)
- [FTC Docs: Universal IMU](https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html)
- [FTCLib command system](https://docs.ftclib.org/ftclib/command-base/command-system/)
- [Road Runner 1.0 tuning](https://rr.brott.dev/docs/v1-0/tuning/)
