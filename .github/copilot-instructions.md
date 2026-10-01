# Copilot Instructions

This is an FRC (FIRST Robotics Competition) 2026 robot project written in Java with WPILib command-based programming.

## Stack
- GradleRIO 2026.2.1 (build with `./gradlew build`, deploy with `./gradlew deploy`, simulate with `./gradlew simulateJava`)
- WPILib command-based framework (`edu.wpi.first.wpilibj2.command`)
- CTRE Phoenix 6 swerve (`com.ctre.phoenix6.swerve`), with generated constants in `frc/robot/generated/TunerConstants.java` (do not hand-edit; regenerate with Tuner X)
- Choreo (`ChoreoLib2026`) for autonomous trajectories; trajectory files live in `src/main/deploy/choreo`
- PhotonVision (`photonlib`) for vision, see `Vision.java`
- Phoenix 6 Orchestra for playing `.chrp` songs from `src/main/deploy`

## Layout
- `src/main/java/frc/robot/RobotContainer.java`: subsystem construction, controller bindings, auto chooser
- `src/main/java/frc/robot/Robot.java`: TimedRobot lifecycle
- `src/main/java/frc/robot/subsystems/`: `CommandSwerveDrivetrain`, `Arms`, `Shoot` (shooter and feeder), `Intake`, `Pneumatics`, `song`
- `src/main/java/frc/robot/Constants.java`: shared constants
- `src/main/java/frc/robot/util/`: helpers

## Controls
- Driver: `Joystick` on port 0 (field-centric swerve, speed scaled by throttle axis 7)
- Operator: `CommandXboxController` on port 1 (shooter, feeder, intake, arms, pneumatics, songs)

## Conventions
- Each mechanism is a `SubsystemBase` subclass. Hardware access stays inside the subsystem; `RobotContainer` only calls its methods.
- Bind buttons with `Trigger`/`CommandXboxController` in `configureBindings()`. Prefer `Commands.*` factories or subsystem command methods over new `Command` classes for simple actions.
- Always pass the owning subsystem as a requirement to `Commands.run`, `runOnce`, and `startEnd`.
- Use WPILib units (`edu.wpi.first.units`) for physical quantities, and put tunable numbers in `Constants.java` rather than inline magic numbers.
- Autos use Choreo `AutoFactory` routines. Event markers in trajectories (`atTime("...")`) and `autoFactory.bind("...")` names are case-sensitive and must match exactly.
- Do not read driver station state (such as alliance) in static initializers; it is not available until the Driver Station connects.
- Never block in `periodic()` or command `execute()`; no `Thread.sleep` or long loops.
- Match the existing style: 4-space indentation, `camelCase` members, `PascalCase` classes.

## Safety
This code drives a real robot. Keep motor outputs bounded, make sure every mechanism has a stop path when its command ends, and do not change CAN IDs, swerve constants, or current limits without being asked.
