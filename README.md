# Command-Kitbot-Starter-SystemCharacterizationFlywheel

*Advanced Robotics — Command-Based Kitbot Unit, Track CF (Characterize the Flywheel).*

## What this is

This is the starter code for **Track CF**, an additional/optional track on top of whichever main track (Feature Build, Autonomous, or Trajectories) your team already chose. It's built on the same kitbot codebase used elsewhere in the unit — drivetrain, intake, loader, flywheel all present — but this track's actual subject is the **Flywheel**, which is a real Kraken X60/TalonFX already installed and functioning on the main competition robot, not a separate practice rig.

Right now the Flywheel only runs open-loop: you tell it a duty cycle, and it spins at whatever speed that duty cycle happens to produce. Your job in this track is to stop guessing gains and instead **measure how the flywheel actually responds**, using WPILib's SysId tooling, and then use those measurements to implement real closed-loop RPM control. That's the whole arc: characterize the real hardware, then close the loop on it.

## The Flywheel today

`Flywheel.java` is intentionally a stripped-down starting point, not the finished subsystem. What's already there:

- One `TalonFX`, controlled through a `DutyCycleOut` request — open-loop percent output only, nothing closed-loop.
- A stored duty-cycle target (`m_currentDutyCycle`, default 0.4, in steps of 0.2, capped at 0.8 — see `IOConstants`) that `runShooterCommand()` applies continuously.
- `increaseShooterSpeedCommand()` / `decreaseShooterSpeedCommand()` to step that target up or down, plus `setDutyCycleCommand(double)` to jump straight to a specific value (useful for holding a fixed test point).
- `stopShooterCommand()`.
- Telemetry already flowing to SmartDashboard every cycle: commanded duty cycle, applied voltage, velocity (RPS and RPM), position, and stator current — a baseline you can watch (and record) by hand before any tuning happens.

What's **not** there yet, on purpose:

- No `SimpleMotorFeedforward`, no `PIDController`, no closed-loop velocity control of any kind.
- No `SysIdRoutine` and no CTRE `SignalLogger` wiring anywhere in the project — nothing is currently capturing data in a form SysId's analysis tool can use.
- `kFlywheel_kS` / `kFlywheel_kV` / `kFlywheel_kA` / `kFlywheel_kP` / `kFlywheel_kI` / `kFlywheel_kD` all sit at `0.0` in `Constants.java` — explicit placeholders waiting on real measurements, not tuned values.
- `getShooterSpeedRPS()` / `getShooterSpeedRPM()` currently report the motor's own rotation. That's the same as the flywheel's rotation here (the integrated encoder drives it directly), but it's worth noticing now — it's exactly the kind of thing that quietly breaks gains later if a gear ratio is ever introduced between motor and mechanism.

## The goal

In broad strokes, not a step list (the actual task-by-task breakdown lives in the challenge set handout, not here):

1. **Characterize** the real Flywheel with a `SysIdRoutine`, logged through CTRE's `SignalLogger` — for a TalonFX-controlled mechanism, signals log automatically once the logger is running, no manual per-cycle logging callback needed. Run the four standard tests (quasistatic/dynamic, forward/reverse) once each, then run the exported log through WPILib's SysId analysis tool to get real kS/kV/kA and a suggested kP.
2. **Close the loop**: implement closed-loop RPM control using those measured constants, and verify it against the open-loop behavior already in this starter as a before/after baseline.

## Files you'll touch

- **`Flywheel.java`** — the main site of change. The SysId routine and SignalLogger calls get added here, and this is where the open-loop duty-cycle path eventually gets replaced (or supplemented) by closed-loop control.
- **`Constants.java`** — once you have real numbers, the `kFlywheel_kS/kV/kA/kP/kI/kD` placeholders in `IOConstants` get filled in.
- **`RobotContainer.java`** — likely a small addition, not a rewrite: the SysId routine's quasistatic/dynamic test commands need to be bound to controller buttons somewhere to actually run them.
- **Your Concept Mastery Log** — not code, but part of the deliverable for this track, same as every other track in this unit.

## Hardware

> This track runs on the real, shared competition robot — not a separate practice board. Check with your instructor about scheduling hardware time before you start running anything, the same as you would for any track that touches the one physical robot.

| Mechanism | Hardware |
|---|---|
| Drivetrain | 4× REV SparkMax-driven brushless motors (2 per side, leader + follower); CTRE CANcoder on the right side; CTRE Pigeon 2.0 gyro |
| Intake | 1× CTRE Kraken X60 (integrated TalonFX), open-loop duty cycle |
| Loader | 1× REV NEO (SparkMax, brushless), open-loop duty cycle, three speeds depending on direction |
| **Flywheel (this track)** | **1× CTRE Kraken X60 (integrated TalonFX)**, integrated rotor encoder, coast neutral mode, 60 A current limit |

## CAN bus map

| Device | CAN ID | Constant |
|---|---|---|
| Drive — left leader | 11 | `kLeftLeaderId` |
| Drive — left follower | 8 | `kLeftFollowerId` |
| Drive — right leader | 10 | `kRightLeaderId` |
| Drive — right follower | 7 | `kRightFollowerId` |
| Drive — right-side CANcoder | 4 | `kRightEncoderID` |
| Pigeon 2.0 gyro | 5 | `kPigeon2ID` |
| **Flywheel motor** | **9** | `kFlywheelMotorID` |
| Intake motor | 12 | `kIntakeMotorID` |
| Loader motor | 19 | `kLoaderMotorID` |

## Current bindings

Driver controller is a `CommandXboxController` on port 0 (`kDriverControllerPort`).

| Input | Command |
|---|---|
| Default (always running) | Split-stick arcade drive — left stick Y for throttle, right stick X for turning |
| `A` button | Runs the intake (`IntakeClass.runIntakeCommand()`) |

Nothing is bound to the Flywheel or Loader yet. That's expected here — this track is about characterizing the mechanism, not wiring it into gameplay, so the SysId test commands (once you build them) are the first thing that'll actually need a button.

## Before you start

- Run each of the four SysId tests — quasistatic forward, quasistatic reverse, dynamic forward, dynamic reverse — **exactly once**. Re-running a test corrupts the log and throws off the analysis tool's results.
- Know where the Driver Station's disable button (Enter) is before you deploy anything, same as always — more so here, since you're commanding open-loop voltage/duty cycle directly to real hardware.

## Where to start reading

These are required reading before you start coding against them, not a citation list:

- [WPILib SysId — Introduction](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html) — what SysId actually measures, and why it beats guessing
- [WPILib SysId — Creating an Identification Routine](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html)
- [CTRE — Plumbing & Running SysId (SignalLogger)](https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/wpilib-integration/sysid-integration/plumbing-and-running-sysid.html) — how SysId actually runs against real CTRE hardware
- [WPILib — Introduction to DC Motor Feedforward](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html)
- [WPILib — Introduction to PID](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html)
- [WPILib — Combining Feedforward and PID](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/combining-feedforward-feedback.html)