// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

//import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Intake;


public class RobotContainer {
private final AutoFactory autoFactory;

    public static Joystick joystick = new Joystick(0);
    public static XboxController joystick2 = new XboxController(1);
    // Shooter subsystem
    

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
    // private final SwerveRequest.FieldCentric autoDriveRequest = new SwerveRequest.FieldCentric()
     //.withDriveRequestType(DriveRequestType.Velocity);
        // Arms/Shooter/Intake subsystems - declared here and constructed in constructor
        public final Arms arms;
        public final Shoot shoot;
        public final Intake intake;
        
    public RobotContainer() {
        // Construct subsystems here to ensure proper initialization order
        this.arms = new Arms();
        this.shoot = new Shoot();
        this.intake = new Intake();
         autoFactory = new AutoFactory(
            drivetrain::getPose, // A function that returns the current robot pose
            drivetrain::resetPose, // A function that resets the current robot pose to the provided Pose2d
            this::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            drivetrain // The drive subsystem
        );
                // ensure subsystems are constructed and default commands registered
        configureBindings();
        
    }
    private void followTrajectory(SwerveSample sample) {

        ChassisSpeeds speeds = drivetrain.parseTrajectory(sample);
        drivetrain.setControl(drive.withVelocityX(speeds.vxMetersPerSecond)
        .withVelocityY(speeds.vyMetersPerSecond)
        .withRotationalRate(speeds.omegaRadiansPerSecond));
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
        
        //Intake
        new JoystickButton(joystick2, XboxController.Button.kX.value)
    .toggleOnTrue(
        Commands.startEnd(
            () -> intake.start(),   
            () -> intake.stop(),    
            intake
        )
    );
    // Shooter runs based on right trigger value
        shoot.setDefaultCommand(
    Commands.run(
        () -> {
            double speed = joystick2.getRightTriggerAxis();
            
                shoot.setSpeed(speed);
                shoot.stop();
        },
        shoot
    )
);

                arms.setDefaultCommand(
    Commands.run(
        () -> {
            double value = joystick2.getLeftX();

            // Optional deadband to prevent drift
            if (Math.abs(value) < 0.05) {
                value = 0.0;
            }

            arms.setSpeed(value);
        },
        arms
    )
);
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                //drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
              //  drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
              //          .withVelocityY(0)
              //          .withRotationalRate(0))
              //          .withTimeout(5.0),
              autoFactory.resetOdometry("NewPath"),
                autoFactory.trajectoryCmd("NewPath"),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle));
    }

        /**
         * Accessor for the arms subsystem.
         */
        public Arms getArms() {
             
                return arms;
        }
        public Shoot getShoot() {
                return shoot;
        }

        public Intake getIntake() {
                return intake;
        }
        
}
