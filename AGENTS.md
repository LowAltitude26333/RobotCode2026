# FTC Robot Development Instructions

This is a FIRST Tech Challenge Robot Controller project. AI assistance is a tool for
students and mentors, not a substitute for understanding the robot, reviewing the
code, or validating behavior on hardware.

## Instruction Priority

- Follow system, developer, and user instructions first. Among repository instruction
  files, the closest `AGENTS.md` that applies to a file overrides this root file.
- Treat this file's project inventory as orientation only. Reinspect the repository
  before every task because OpModes, hardware mappings, and active implementations
  can change.
- When repository evidence and this inventory disagree, report the mismatch and use
  the repository as the source of truth.

## Repository Boundaries

- Robot code belongs primarily in
  `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.
- Make robot-code changes inside `TeamCode` unless the user explicitly requests a
  different module.
- `FtcRobotController` contains the FTC SDK/controller application. Do not modify it
  unless explicitly requested.
- `MeepMeepTesting` is the desktop Road Runner trajectory simulation module. Changes
  there do not verify behavior on the physical robot.
- Do not update Gradle, the Android Gradle Plugin, Java, Android dependencies, the
  FTC SDK, or accept automatic Android Studio upgrades unless explicitly requested.
- Do not add a dependency without explaining why the existing SDK and libraries are
  insufficient.
- Preserve the repository structure and avoid unrelated refactors.
- Preserve user changes in a dirty worktree. Never reset, overwrite, or revert work
  that is unrelated to the request.

## Required Repository Discovery

Before proposing or making code changes:

1. Read every applicable `AGENTS.md` and inspect `git status --short`.
2. Read `settings.gradle`, `TeamCode/build.gradle`, and the relevant package tree.
3. Inspect the complete call path for the requested behavior: OpMode,
   `RobotContainer`, command, subsystem, drive/localizer, constants, and
   `RobotMap` as applicable.
4. Search for OpMode annotations, subsystem and command classes, every affected
   `hardwareMap` lookup, direction/inversion setting, encoder mode, power/position
   limit, timeout, and stop path. Prefer `rg`/`rg --files`.
5. Identify duplicate or competing implementations and determine which one is
   actually instantiated. Do not infer activity from a filename alone.
6. State missing hardware facts and present a short, scoped implementation plan.

Useful discovery commands from the repository root:

```powershell
rg --files TeamCode/src/main/java/org/firstinspires/ftc/teamcode
rg -n --glob '*.java' '@TeleOp|@Autonomous|@Disabled' TeamCode/src/main/java
rg -n --glob '*.java' 'extends SubsystemBase|extends CommandBase|extends SequentialCommandGroup' TeamCode/src/main/java
rg -n --glob '*.java' 'hardwareMap|get\(|setDirection|setInverted|setPosition|setPower|set\(' TeamCode/src/main/java/org/firstinspires/ftc/teamcode
git status --short
```

## Project Architecture Snapshot

Snapshot date: 2026-06-19. This describes code observed in the repository, not the
physical robot configuration.

- The project uses Java, FTC SDK 10.3, FTCLib's command system, Road Runner 1.0,
  FTC Dashboard, and MeepMeep.
- `RobotContainer` composes subsystems and driver bindings. `RobotMap` centralizes
  most competition hardware names and inversions. `LowAltitudeConstants` contains
  Dashboard-tunable shooter, hood, intake, and kicker values.
- `opmodes/teleop` and `opmodes/auto` contain competition, test, and tuning OpModes.
- `subsystems` owns mechanism behavior; `commands` coordinates subsystem actions;
  `oi` owns driver control profiles; `tuning` contains Road Runner tuning utilities;
  `messages` contains Road Runner flight-recorder payloads.
- `MecanumDrive` is the drive used by `DriveSubsystem`. It currently instantiates
  `TwoDeadWheelLocalizer`; Pinpoint, OTOS, three-dead-wheel, drive-encoder, and tank
  implementations also exist but are not thereby active.

### Current OpModes

TeleOp annotations currently register:

- `MainTeleOp` - `Skywalker TeleOp (Manual)`
- `MainTeleOp2` - `Skywalker TeleOp 2`
- `TeleOpApril` - `Skywalker TeleOp w/ April`
- `TeleOpPositionBEta` - `TeleOpBETA 1 ROJO`
- `TeleOpAlignWithPoint` - `boton`
- `TeleOpMotoresPrueba` - `Motores`
- `TeleOpShooter` - `ShooterTeleOpAdaptado` (located under `commands/drivetrain`)
- `SystemCheckOpMode` - `SYSTEM CHECK and TUNING`
- `ShooterTuningOpMode` - `Tuning: Shooter & Systems (Manual)`
- `TestColor` - `Test Color`
- `FTCLibTestOpMode` - `FTCLib Test`
- `TeleOpFieldCentric` - `TeleOpFieldCentric`, currently `@Disabled`

Autonomous annotations currently register:

- `AutonomoBetaPosition` - `Beta`
- `AutonomoOfficialBlue` - `AutonomoOfficialBlue 1`
- `AutonomoOfficialRed` - `AutonomoOfficialRed 1`
- `FullOficialBlue` - `FullOficialBlue 1`
- `FullOficialRed` - `FullOficialRed1`
- `TestShootBurstAuto` - `TEST: Shoot Burst Command`

`TuningOpModes` dynamically registers Road Runner tuning routines, including
`LocalizationTest`, `ManualFeedbackTuner`, and `SplineTest`; annotations are not the
only registration mechanism in this project.

### Current Subsystems and Commands

- `DriveSubsystem`: Road Runner mecanum drive and pose updates.
- `ShooterSubsystem`: dual-motor shooter with encoder feedback, PID/feedforward,
  voltage compensation, filtering, and output clamping.
- `ShooterMotor`: separate manual dual-motor shooter implementation.
- `ShooterHoodSubsystem`: paired servos with inversion and angle clamping.
- `KickerSubsystem`: raw-power kicker motor with brake behavior.
- `IntakeSubsystem`: intake motor with forward, reverse, and stop operations.
- `ColorSubsystem`: REV color sensor classification and telemetry.
- Commands include drive, field-centric drive, precision mode, AprilTag approach,
  pose actions, shooter control/sequences, color detection, burst shooting, and
  turning. `PoseStorage` carries pose state between modes.
- `SkywalkerProfile` and `SkywalkerProfileMotorTest` define driver bindings.

### Hardware Names Observed in Code

Competition names in `RobotMap` are `leftFront`, `rightFront`, `leftBack`,
`rightBack`, `shooterMotorDown`, `shooterMotorUp`, `intakeMotor`, `hoodLeft`,
`hoodRight`, and `kickerMotor`.

Other direct lookups include `imu`, `color`, `left_drive`, `left`, `right`, `par0`,
`par1`, `perp`, `pinpoint`, and `sensor_otos`. Some belong to disabled tests,
alternative drives, or inactive localizers. The two-dead-wheel localizer currently
uses `rightFront` and `rightBack` encoder ports. Confirm all names against the active
Robot Controller configuration before relying on them.

## AI-Assisted Development and Student Ownership

- Use AI to inspect the repository, explain code, locate current primary
  documentation, generate small reviewable changes, analyze logs, and build tests or
  simulation checks.
- Do not treat natural-language output as a specification. Convert the request into
  explicit behavior, units, limits, ownership, and acceptance criteria first.
- A student or responsible human reviewer must be able to explain each nontrivial
  generated change, its data flow, units, failure modes, and how it stops safely.
- Prefer small diffs that match established project patterns. Do not replace working
  architecture merely because generated code uses a more familiar pattern.
- Verify SDK and vendor APIs against the versions and official documentation used by
  this repository. FTC/FRC APIs change, and plausible generated code may use stale
  classes or semantics.
- Never use AI output as evidence for hardware specifications, game rules, wiring,
  dimensions, or device configuration. Verify those with the robot configuration,
  manufacturer documentation, current game manual, or measurements.
- Never deploy opaque generated code directly to an enabled robot. Compilation,
  simulation, and log analysis reduce risk but do not replace code review and staged
  hardware testing.
- In the final response, explain important logic and tradeoffs so the team can own,
  debug, and maintain the result.

## Hardware Safety

- Never invent hardware device names or silently substitute a similar device.
- Ask for the Robot Controller configuration when required names cannot be verified.
- Do not assume motor directions, encoder signs/resolution, gear ratios, wheel or
  pulley diameters, servo travel, current limits, IMU orientation, odometry geometry,
  camera pose, or mechanism dimensions.
- Never remove or weaken safety limits, timeouts, current limits, interlocks,
  watchdogs, or emergency-stop behavior.
- Every motorized mechanism must have an explicit zero-power path when a command is
  interrupted and when its OpMode stops. Confirm FTCLib command requirements and
  default-command behavior rather than assuming subsystem methods will be called.
- Mechanisms must have explicit software ranges consistent with verified physical
  limits. Raw servo positions and Dashboard-tunable limits require special scrutiny.
- Avoid unbounded loops, blocking command execution, and sleeps that can prevent
  scheduler updates, stop requests, or actuator shutdown.
- Keep test areas clear, restrain or raise the robot when appropriate, announce
  enabling, maintain immediate access to Stop/E-stop, and begin at reduced power.
- Validate in stages: static review, compilation, simulation where meaningful,
  robot-disabled inspection, robot-on-blocks/physically restrained testing, then
  controlled low-power field testing.
- Clearly label every behavior that still requires physical validation. Never claim
  physical safety or correctness from compilation or simulation alone.

## Implementation Rules

- Follow the existing FTCLib command/subsystem ownership model and Road Runner APIs.
- Keep hardware ownership in the established subsystem or drive class; avoid mapping
  the same actuator independently in multiple active components.
- Commands must declare requirements correctly and place actuator cleanup in
  interruption/end paths where appropriate.
- Do not add blocking loops to command-based OpModes or commands.
- Do not change the active localizer, drive type, motor directions, tuning constants,
  or coordinate conventions as incidental cleanup.
- Do not enable disabled, test, system-check, or tuning OpModes unless explicitly
  requested.
- Keep units explicit in names or comments when ambiguity could cause physical harm.
- Preserve safety clamps in normal methods and Road Runner `Action` adapters; every
  alternate control path must enforce the same limits.

## Verification Workflow

After modifying code:

1. Review the complete diff and all changed files for accidental scope expansion.
2. Recheck affected hardware names, units, bounds, command requirements, interruption
   behavior, and OpMode stop paths.
3. Run focused static checks or tests and Road Runner/MeepMeep simulation when they
   can exercise the changed behavior.
4. From Windows PowerShell at the repository root, run:

   ```powershell
   .\gradlew.bat assembleDebug
   ```

5. Fix compilation errors caused by the changes. Report unrelated or environmental
   failures without modifying build tooling unless requested.
6. Summarize files changed, behavior implemented, tests/build results, remaining
   assumptions, and an explicit physical validation checklist.

## Research Basis

These sources inform the AI workflow; they do not override FTC rules, official SDK
documentation, this repository, or verified robot configuration.

- [FTC Docs: Android Studio OpMode structure](https://ftc-docs.firstinspires.org/en/latest/programming_resources/android_studio_java/opmode/opmode.html)
- [FTC Docs: hardware configuration](https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html)
- [FTC Docs: Universal IMU interface and orientation](https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html)
- [FTCLib command system](https://docs.ftclib.org/ftclib/command-base/command-system/)
- [Road Runner 1.0 tuning documentation](https://rr.brott.dev/docs/v1-0/tuning/)
- [Team 254: The Next Revolution - AI in FRC](https://www.chiefdelphi.com/t/2026-championship-conference-presentation-the-next-revolution-ai-in-frc-by-254/519529), including its context-rich agent, simulation, and log-analysis workflow
- [Team 254 presentation recording](https://www.youtube.com/watch?v=oTcimMwxRoM)
- [Team 4414 2026 Technical Binder](https://2026.team4414.com/), which documents AI-agent attempts followed by human review, logs as debugging context, and reusable agent skills
- [Chief Delphi: Lowering the Barrier to Code](https://www.chiefdelphi.com/t/lowering-the-barrier-to-code-ais-role-in-frc-robotics/503634) and [Student written code](https://www.chiefdelphi.com/t/student-written-code/520752), community discussions emphasizing review, current documentation, hardware safety, and student understanding
