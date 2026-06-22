# FTC Robot Code Critical Findings

## Review scope

This report reviews `backup/main-before-split-20260619` at commit `e42e7cf`, the
newest complete robot-source snapshot available locally on 2026-06-19. The current
`main` branch is documentation-only and was not treated as robot source.

The snapshot builds successfully with `gradlew.bat assembleDebug`. Compilation does
not validate physical behavior, wiring, calibration, game rules, or mechanism safety.

## Critical findings

### 1. Red autonomous requests a 90-radian turn

Road Runner's `turn()` argument is radians, but two registered autonomous modes call
`.turn(90)`. This requests approximately 14.3 complete rotations instead of a
90-degree turn and can cause uncontrolled spinning or collisions.

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/FullOfficialRed2.java:82`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/FullOficialRed.java:74`

Replace the literal only after confirming the intended direction. A 90-degree value
must be expressed as `Math.toRadians(90)` or `Math.PI / 2`.

### 2. Active odometry uses powered drive-motor encoders as dead wheels

`MecanumDrive` commands `rightFront` and `rightBack` as drive motors, while
`TwoDeadWheelLocalizer` reads those same devices as parallel and perpendicular dead
wheels. Drive-wheel encoder motion does not satisfy the two-dead-wheel model,
especially during mecanum strafing and rotation. Autonomous pose estimation and
field-centric control can therefore be fundamentally wrong.

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumDrive.java:235`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumDrive.java:265`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TwoDeadWheelLocalizer.java:53`

The physical encoder layout and hub ports must be verified before selecting or
correcting the localizer.

### 3. Interrupted commands can leave motors powered

`ActionCommand` has no `end(boolean interrupted)` implementation. Road Runner actions
zero the drivetrain only when they finish naturally, so cancellation can leave the
last drive command applied.

`ShootBurstCommand` starts the kicker with an `InstantCommand` and then waits 632 ms.
If the group is interrupted during that wait, the active `WaitCommand` ends but no
code stops the kicker motor.

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/ActionCommand.java:34`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/auto/ShootBurstCommand.java:65`

Every interruption and OpMode stop path must explicitly command zero power.

### 4. Three registered autonomous modes fail during initialization

`Beta`, `nada 1`, and `nada1` construct a `ParallelCommandGroup` containing a
permanent `ShooterPIDCommand` alongside a sequence containing `ShootBurstCommand`.
Both commands require `ShooterSubsystem`. FTCLib 2.1.1 rejects duplicate subsystem
requirements in a parallel group by throwing `IllegalArgumentException`.

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/AutonomoBetaPosition.java:73`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/FullOficialBlue.java:86`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/FullOficialRed.java:81`

These OpModes should not be selected until command ownership is corrected and their
initialization is tested on a robot-disabled system.

## High-severity findings

### 5. Shooter readiness has no timeout or failure path

`ShootBurstCommand` uses `WaitUntilCommand(shooter::isReady)` before and between
shots. An encoder fault, jam, low battery, incorrect ratio, or unreachable RPM causes
the autonomous sequence to wait forever. Parking and final shutdown are then never
reached, while the shooter and intake may remain active.

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/auto/ShootBurstCommand.java:34`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/auto/ShootBurstCommand.java:68`

Add a measured timeout and a failure branch that stops the shooter, kicker, and
intake before continuing to a safe park or ending the routine.

### 6. The TeleOp emergency stop is not latched

The Back-button handler calls `CommandScheduler.cancelAll()` and stops several
mechanisms. FTCLib automatically reschedules default commands after cancellation, so
the drive and automatic turret search can resume on a later scheduler cycle. The
turret is also missing from the explicit stop list.

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/oi/SkywalkerProfile.java:92`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/MainTeleOp.java:74`

An emergency stop must latch until an intentional reset and prevent all default
commands from producing actuator output.

### 7. Shooter Stop controls can re-enable the shooter

Both enabled tuning OpModes stop the shooter when a button is pressed, but their next
loop restores the configured target whenever measured RPM is still nonzero. Because
the flywheel coasts, a Stop command can re-arm the motor immediately.
`SystemCheckOpMode` also recreates the PID/feedforward controllers every cycle,
discarding controller state and invalidating integral or derivative tuning.

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/ShooterTuningOpMode.java:86`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/SystemCheckOpMode.java:90`

Use an explicit enabled state controlled by the operator, and reload controllers only
when tunable constants actually change.

### 8. Blue autonomous has a 20-inch path discontinuity

In `AutonomoOfficialBlue`, `path4` ends at `(10, -70)` inches, but `path5` declares
its start at `(10, -50)` inches. Starting the next action with a false pose creates an
immediate 20-inch controller error and can produce an aggressive correction.

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/AutonomoOfficialBlue.java:75`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/AutonomoOfficialBlue.java:82`

Every trajectory's declared start must match the previous trajectory's planned end,
and the complete chain must be simulated before restrained hardware testing.

### 9. Current red autonomous modes store the blue alliance state

`AutonomoOfficialRed2` and `FullOfficialRed2` assign
`PoseStorage.isRedAlliance = false`. TeleOp uses this value to select aiming angles,
so starting TeleOp after red autonomous selects blue targets until manually toggled.

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/AutonomoOfficialRed2.java:38`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto/FullOfficialRed2.java:39`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/oi/SkywalkerProfile.java:111`

## Additional design problems

### Drive normalization ignores negative magnitude

`MecanumDrive.setDrivePowers()` normalizes using the largest positive wheel value,
not the largest absolute value. When the dominant wheel command is negative, SDK
clipping can distort the requested translation and rotation.

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumDrive.java:280`

### Primary TeleOp bypasses configured drive behavior

`RobotContainer` installs a field-centric default command with precision scaling.
`MainTeleOp` replaces it with a direct `RunCommand` that ignores
`PoseStorage.isPrecisionMode`. The Driver Station can display precision mode even
though the primary TeleOp does not apply it.

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotContainer.java:81`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/MainTeleOp.java:58`

### Turret vision has no target selection or deterministic cleanup

The turret follows the first AprilTag detection without filtering tag IDs. When no
tag is visible, it immediately sweeps at hard-coded power `0.3`. The OpModes do not
close `VisionPortal`, and the turret constructor resets its encoder without a homing
sensor, assuming the mechanism was manually centered.

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/TurretFollowTagCommand.java:28`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/TurretFollowTagCommand.java:57`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/TurretSubsystem.java:25`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/MainTeleOp.java:47`

### Autonomous stiction workaround does nothing

Several autonomous modes call `setTargetRPM(2900)`, sleep for 100 ms during
initialization, and then stop the shooter. `setTargetRPM()` only stores a target;
motor output is applied by `ShooterSubsystem.periodic()`, which is not running during
that initialization sleep. The motor never receives the intended startup pulse.

## Hard-coded values requiring physical validation

The following values may be legitimate calibrations, but repository evidence does
not prove them safe or correct:

- Turret travel limits: `-200` to `+200` encoder ticks.
- Turret search power: `0.3`; tracking power limit: `0.5`.
- Shooter encoder: `28` ticks/revolution; maximum: `6000 RPM`.
- Shooter external ratio: `2.0`; presets: `2450` to `3600 RPM`.
- Hood logical range: `5` to `207` degrees on a declared 300-degree servo range.
- Kicker powers: `+0.7` and `-0.7`; open-loop timings from 60 to 632 ms.
- IMU hub orientation: logo right, USB up.
- Camera name: `Webcam 1`; dormant container code uses `Webcam`.
- Kicker hardware name: `kickerM otor`, including the embedded space.
- Shooter hardware count: one active motor while `Shooter2` remains declared.

Do not change these values by inference. Verify the Robot Controller configuration,
wiring, motor directions, encoder signs, gear ratios, servo linkage limits, turret
hard-stop clearance, camera orientation, and intended AprilTag IDs.

## Repository mismatch

`AGENTS.md` lists `AutonomoOfficialRed` as registered, but
`AutonomoOfficialRed.java` is entirely inside a block comment at `e42e7cf` and does
not produce an OpMode. The source ref is the authority for this review.

## Recommended remediation order

1. Disable or avoid affected autonomous and tuning modes until actuator shutdown and
   initialization failures are corrected.
2. Fix the 90-radian turn, path discontinuity, alliance state, and parallel command
   ownership errors.
3. Add interruption cleanup, shooter timeouts, failure shutdown, and a latched
   emergency stop.
4. Confirm the physical odometry architecture and select a matching localizer.
5. Validate turret, shooter, hood, kicker, IMU, and camera assumptions at reduced
   power with immediate Stop access.
6. Rebuild, simulate trajectories, inspect the robot while disabled, then perform
   restrained/on-blocks tests before controlled field testing.
