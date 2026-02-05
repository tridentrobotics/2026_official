// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

//import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
        
//    private final CommandXboxController joystick2 = new CommandXboxController(1);
        
    public final Joystick joystick = new Joystick(0);

    private double MaxSpeed; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.4).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                     // angular velocity
    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        // Arms subsystem - controls the robot arm motor
        public final Arms arms = new Arms();

    public RobotContainer() {
                // ensure subsystems are constructed and default commands registered
        configureBindings();
        
    }

    private void configureBindings() {
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {

                    MaxSpeed = Math.abs(joystick.getRawAxis(5)) *
                            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

                    return drive
                            .withVelocityX(joystick.getY() * MaxSpeed)
                            .withVelocityY(joystick.getX() * MaxSpeed)
                            .withRotationalRate(joystick.getZ() * MaxAngularRate);
                }));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        new JoystickButton(joystick, 6).whileTrue(drivetrain.applyRequest(() -> brake));
        new JoystickButton(joystick, 4).whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getY(), -joystick.getX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
       // new JoystickButton(joystick, 10)
       //         .and(new JoystickButton(joystick, 11))
       //         .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));

       // new JoystickButton(joystick, 10)
       //         .and(new JoystickButton(joystick, 12))
       //         .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

        // SysId – Quasistatic
        new JoystickButton(joystick, 8)
                .and(new JoystickButton(joystick, 9))
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

        new JoystickButton(joystick, 8)
                .and(new JoystickButton(joystick, 7))
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Field-centric reset
        new JoystickButton(joystick, 2)
                .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5.0),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle));
    }

        /**
         * Accessor for the arms subsystem.
         */
        public Arms getArms() {
                return arms;
        }
}
