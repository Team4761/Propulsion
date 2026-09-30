# Propulsion

> [!WARNING]
> **Made by Claude (AI) and NOT yet reviewed by a human.**
> This code was written by Claude Code from the CompetitionBot2026 motor utilities. It has **not**
> been compiled against the real vendor libraries, run on a robot, or reviewed by a team member.
> The pure-math tests (`MotorMathTest`) pass; the Phoenix 6 / REVLib calls are unverified.
> Treat it as a draft: review it, build it, and test it on hardware before relying on it at an
> event. Remove this notice once someone has done that.

Team 4761's reusable robot code library. Anything we copy from one season's robot into the next belongs
here instead.

Right now that is the motor wrappers:

| Class | Hardware | Notes |
| --- | --- | --- |
| `SmartKrakenMotor` | Kraken X60/X44 (any TalonFX) | Phoenix 6 |
| `SmartVortexMotor` | NEO Vortex on a SPARK Flex | REVLib |
| `SmartMotor` | interface both implement | swap motors without touching commands |
| `MotorMath` | pure math (wrapping, limits, gear ratios) | unit tested |

## Installing in a robot project

The library ships as a WPILib vendordep served by [JitPack](https://jitpack.io/#Team4761/Propulsion).

**VS Code:** `WPILib: Manage Vendor Libraries` → `Install new library (online)` → paste

```
https://raw.githubusercontent.com/Team4761/Propulsion/main/Propulsion.json
```

**By hand:** copy `Propulsion.json` into your project's `vendordeps/` folder.

GroundZero (our yearly template) already includes it.

The library declares WPILib, Phoenix 6 and REVLib as `compileOnly`, so your robot project must
also have the Phoenix 6 and REVLib vendordeps installed (GroundZero does).

## Using the motors

```java
import frc.team4761.propulsion.motors.MotorMode;
import frc.team4761.propulsion.motors.SmartKrakenMotor;
import frc.team4761.propulsion.motors.SmartVortexMotor;

SmartKrakenMotor turret = SmartKrakenMotor.Builder.newInstance()
    .port(25)
    .PID(0.5, 0.0, 0.0)            // position gains
    .gearRatio(185.0 / 28.0)       // motor rotations per turret rotation
    .angleLimits(-80.0, 80.0)      // soft limits in turret degrees
    .mode(MotorMode.CONTINUOUS)
    .statorCurrentLimit(40.0)
    .build();

turret.set(45.0);                  // go to 45 degrees
turret.turn(-10.0);                // 10 degrees back from wherever it is
turret.isAtAngle(45.0, 1.0);

SmartVortexMotor roller = SmartVortexMotor.Builder.newInstance()
    .canId(43)
    .velocityPID(0.0002, 0.0, 0.0)
    .velocityFF(0.1, 0.0017)
    .build();

roller.setSpeed(3000.0);           // mechanism RPM
roller.setSpeedPercent(0.8);       // open loop
roller.stopTurning();
```

Units: everything is in **mechanism** degrees / RPM (after the gear ratio) unless the method name
has `Raw` in it. `gearRatio` is motor rotations per mechanism rotation.

Closed-loop slots: position uses slot 0, velocity uses slot 1. If you only call `PID(...)`, those
gains are used for both.

## Changes from the 2026 robot's `frc.robot.util` versions

Drop-in except for the package name (`frc.team4761.propulsion.motors`) and `SmartKrakenMotor.MotorMode` → `MotorMode`.

- **Bug:** builders defaulted the output range to `[0, 0]` and angle limits to `[0, 0]`, so a motor
  built without `outputRange(...)` / `angleLimits(...)` could not move. Defaults are now full output
  and no limits.
- **Bug:** `angleLimits(-1, -1)` was a magic "no limits" value, which made a real limit of -1
  degrees impossible. Limits are now off unless you call `angleLimits`.
- **Bug:** WRAPPED mode used Java's `%`, which returns negative angles, and then commanded that
  angle directly so the turret could spin the long way around. It now takes the shortest path.
- **Bug:** `set()` checked the limits against the unwrapped angle but commanded the wrapped one.
- **Bug:** `SmartVortexMotor.setSpeedPercent` multiplied the percent by the gear ratio, so any
  reduction above 1:1 slammed to full output.
- **Bug:** `SmartVortexMotor.setSpeed` used closed-loop velocity without ever configuring gains, so
  it did nothing. Gains and feedforward are now configurable.
- Kraken velocity control allocated a new `VelocityDutyCycle` every loop; it now reuses one.
- Added: inversion, brake/coast, current limits, CAN bus name, velocity feedforward,
  `resetAngle`, `isAtAngle`, `getWrappedAngle`, Vortex position control and `getSpeedRPM`/`getAngle`,
  and `getMotor()` as an escape hatch.
- Config failures and limit violations are reported to the Driver Station instead of
  `java.util.logging` (which nobody reads on the RIO).

## Releasing a new version

The vendordep currently pins commit `10b6aed7cd` (JitPack builds any commit hash). Once the code
has been reviewed, cut a real release:


1. Bump the versions in `build.gradle` (WPILib / Phoenix / REVLib) at the start of each season.
2. Commit, then create a GitHub release with a tag like `v0.1.0` (or `git tag v0.1.0 && git push origin v0.1.0`).
3. Open `https://jitpack.io/#Team4761/Propulsion` and click **Get it** on the tag to trigger the
   build (it also builds on first download).
4. Update `version` in `Propulsion.json` (both the top-level `version` and the dependency
   `version`, which is the tag name) and bump `frcYear` when the season changes.

## Building locally

```
./gradlew build
```

Runs the `MotorMath` unit tests. The wrappers themselves need hardware (or sim) to test.
