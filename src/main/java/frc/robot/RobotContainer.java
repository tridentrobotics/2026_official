// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.song;
import pabeles.concurrency.IntOperatorTask.Max;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Shoot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;


public class RobotContainer {
    
    private final AutoFactory autoFactory;
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    public static class alliance {
    public static final AllianceStationID Alliance = DriverStation.getRawAllianceStation(); //to use import frc.robot.RobotContainer.alliance;
    }
    // At the top of the class, replace the joystick declaration block:

public static Joystick joystick = Constants.OperatorConstants.useController ? null : new Joystick(0);
public static CommandXboxController driverController = Constants.OperatorConstants.useController ? new CommandXboxController(0) : null;
public static CommandXboxController controller = new CommandXboxController(1);

    public final song m_song = new song();
    private final Pneumatics Pneumatics = new Pneumatics();

    private double MaxSpeed;
    private double MaxAngularRate;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

                 //  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        // Heading-hold controller: when the rotation joystick is released (near zero),
        // hold the robot heading by applying a small corrective rotational rate.
        private final PIDController m_headingHoldController = new PIDController(20, 0.0, 0.0);
        private boolean m_headingHoldActive = false;
        private double m_targetHeadingRadians = 0.0;
        // joystick Z deadband for considering 'released'
        private static final double kHeadingDeadband = 0.05;

    public final Arms arms;
    public final Shoot shoot;
    public final Intake intake;

    public RobotContainer() {
        System.out.println(alliance.Alliance);
        this.arms = new Arms();
        this.shoot = new Shoot();
        this.intake = new Intake();

        autoFactory = new AutoFactory(
                drivetrain::getPose,
                drivetrain::resetPose,
                this::followTrajectory,
                true,
                drivetrain
        );

        autoFactory.bind("shoot", Commands.run(() -> shoot.setSpeed(1.0), shoot)
                .withTimeout(2.0)
                .andThen(Commands.runOnce(() -> shoot.stop(), shoot)));

        configureAutos();
        configureBindings();
    }

    private Command shootSequence() {
        return Commands.run(() -> shoot.setSpeed(1.0), shoot)
                .withTimeout(2.0)
                .andThen(Commands.runOnce(() -> shoot.stop(), shoot));
    }


    private void configureAutos() {
    // Blue Top
     var blueTop = autoFactory.newRoutine("Blue 1");                    //here
    var blueTop1 = blueTop.trajectory("Blue_1_pt1");       
    var blueTop2 = blueTop.trajectory("Blue_1_pt2");
    blueTop.active().onTrue(blueTop1.cmd());
    blueTop1.atTime("shoot").onTrue(shootSequence());
    blueTop1.done().onTrue(blueTop2.cmd());
    blueTop2.atTime("shoot").onTrue(shootSequence());
    blueTop2.atTime("intake_on").onTrue(
        Commands.startEnd(
            () -> intake.start(-.3),
            () -> intake.stop(),
            intake
        )
    );
    autoChooser.setDefaultOption("Blue 1", blueTop.cmd());

    // Blue Mid
    var blueMid = autoFactory.newRoutine("Blue Mid");
    var blueMid1 = blueMid.trajectory("Blue_Mid_1");
    var blueMid2 = blueMid.trajectory("Blue_Mid_2");
    blueMid.active().onTrue(blueMid1.cmd());
    blueMid1.atTime("shoot").onTrue(shootSequence());
    blueMid1.done().onTrue(blueMid2.cmd());
    autoChooser.addOption("Blue Mid", blueMid.cmd());

    // Blue Bot
    var blueBot = autoFactory.newRoutine("Blue Bot");
    var blueBot1 = blueBot.trajectory("Blue_Bot_1");
    var blueBot2 = blueBot.trajectory("Blue_Bot_2");
    blueBot.active().onTrue(blueBot1.cmd());
    blueBot1.atTime("shoot").onTrue(shootSequence());
    blueBot1.done().onTrue(blueBot2.cmd());
    autoChooser.addOption("Blue Bot", blueBot.cmd());

    SmartDashboard.putData("Auto Chooser", autoChooser);
}

    private void followTrajectory(SwerveSample sample) {
        ChassisSpeeds speeds = drivetrain.parseTrajectory(sample);
        drivetrain.setControl(
                drive.withVelocityX(speeds.vxMetersPerSecond)
                        .withVelocityY(speeds.vyMetersPerSecond)
                        .withRotationalRate(speeds.omegaRadiansPerSecond));
    }

    private void songbinds() {
        controller.povUpRight().toggleOnTrue(
                Commands.startEnd(
                        () -> m_song.playSong("rick.chrp"),
                        () -> m_song.stopSong(),
                        m_song
                )
        );

     //   controller.b().toggleOnTrue(
    //            Commands.startEnd(
    //                    () -> m_song.playSong("LoZMain.chrp"),  
   //                     () -> m_song.stopSong(),
    //                    m_song
    //            )
    //    );
        controller.povDown().toggleOnTrue(
                Commands.startEnd(
                        () -> m_song.playSong("LoZdungeonmusic.chrp"),  
                        () -> m_song.stopSong(),
                        m_song
                )
        );
        controller.povUp().toggleOnTrue(
                Commands.startEnd(
                        () -> m_song.playSong("mariobrosmain.chrp"),  
                        () -> m_song.stopSong(),
                        m_song
                )
        );
        controller.povLeft().toggleOnTrue(
                Commands.startEnd(
                        () -> m_song.playSong("doom.chrp"),  
                        () -> m_song.stopSong(),
                        m_song
                )
        );
        controller.povRight().toggleOnTrue(
                Commands.startEnd(
                        () -> m_song.playSong("lavendertown.chrp"),  
                        () -> m_song.stopSong(),
                        m_song
                )
        );

    }

    private void configureBindings() {
    songbinds();
    controller.a()
        .onTrue(Commands.runOnce(() -> Pneumatics.toggleSolenoids(), Pneumatics));
        
    if (Constants.OperatorConstants.useController) {
        // --- Xbox controller on port 0 ---
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                MaxSpeed = 1;

                return drive
                        .withVelocityX(-driverController.getLeftX() * MaxSpeed)
                        .withVelocityY(-driverController.getLeftY() * MaxSpeed)
                        .withRotationalRate(-driverController.getRightX() * MaxAngularRate);
            }));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

       

        // Point wheels on right bumper
        driverController.rightBumper().whileTrue(
                drivetrain.applyRequest(() ->
                        point.withModuleDirection(new Rotation2d(
                                -driverController.getLeftY(),
                                -driverController.getLeftX()))
                ));

        // Reset field-centric heading on start button
        driverController.a()
                .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

       

    } else {
        // --- Joystick on port 0 (original behavior, untouched) ---
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                MaxSpeed = Math.abs(joystick.getRawAxis(7))
                * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
                MaxAngularRate = RotationsPerSecond.of(joystick.getRawAxis(5)).in(RadiansPerSecond);
                if (joystick.getRawAxis(5) == -1) {
                        MaxAngularRate =RotationsPerSecond.of(joystick.getRawAxis(5)).in(RadiansPerSecond);;
                }

                                // Rotation input from Z axis. If it's near zero, engage heading-hold
                                // which counters drift by steering toward the last target heading.
                                double joystickZ = joystick.getZ();
                                double rotationalRate;
                                if (Math.abs(joystickZ) > kHeadingDeadband) {
                                        // Driver is commanding rotation: pass-through and disable hold
                                        m_headingHoldActive = false;
                                        rotationalRate = -joystickZ * MaxAngularRate;
                                } else {
                                        // Driver released rotation stick: enable/maintain heading hold
                                        double currentHeading = drivetrain.getPose().getRotation().getRadians();
                                        if (!m_headingHoldActive) {
                                                // capture target heading when we first enter the deadband
                                                m_targetHeadingRadians = currentHeading;
                                                m_headingHoldActive = true;
                                                m_headingHoldController.reset();
                                        }

                                        // PID controller computes an angular-rate correction (rad/s)
                                        double correction = m_headingHoldController.calculate(currentHeading, m_targetHeadingRadians);
                                        // clamp to allowed max angular rate
                                        rotationalRate = Math.max(-MaxAngularRate, Math.min(MaxAngularRate, correction));
                                }

                                return drive
                                                .withVelocityX(-joystick.getY() * MaxSpeed)
                                                .withVelocityY(-joystick.getX() * MaxSpeed)
                                                .withRotationalRate(rotationalRate);
            }));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        new JoystickButton(joystick, 4).whileTrue(
                drivetrain.applyRequest(() ->
                        point.withModuleDirection(new Rotation2d(-joystick.getY(), -joystick.getX()))
                ));

        new JoystickButton(joystick, 2)
                .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }

    // Everything below is unchanged regardless of driver input mode:

    controller.x().whileTrue(
            Commands.startEnd(
                    () -> intake.start(-.3),
                    () -> intake.stop(),
                    intake
            )
    );


    controller.b().whileTrue(
            Commands.startEnd(
                    () -> intake.start(.3),
                    () -> intake.stop(),
                    intake
            )
    );

    // While left bumper is held, run the feeder only
    controller.leftBumper().whileTrue(
            Commands.startEnd(
                    () -> shoot.feed(),
                    () -> shoot.stopFeed(),
                    shoot
            )
    );

    // Directional nudge buttons for operator controller:
    // Y -> forward, Right Bumper -> right, Start -> back, Back -> left
    // while held, apply a fixed field-centric velocity request
    final double nudgeSpeed = 0.6 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    new JoystickButton(joystick, 5).whileTrue(
            drivetrain.applyRequest(() ->
                    drive.withVelocityX(nudgeSpeed)
                            .withVelocityY(0)
                            .withRotationalRate(0)
            )
    );

    new JoystickButton(joystick, 6).whileTrue(
            drivetrain.applyRequest(() ->
                    drive.withVelocityX(0)
                            .withVelocityY(nudgeSpeed)
                            .withRotationalRate(0)
            )
    );

    new JoystickButton(joystick, 7).whileTrue(
            drivetrain.applyRequest(() ->
                    drive.withVelocityX(-nudgeSpeed)
                            .withVelocityY(0)
                            .withRotationalRate(0)
            )
    );

    new JoystickButton(joystick, 8).whileTrue(
            drivetrain.applyRequest(() ->
                    drive.withVelocityX(0)
                            .withVelocityY(-nudgeSpeed)
                            .withRotationalRate(0)
            )
    );

    shoot.setDefaultCommand(
    Commands.run(() -> {

        double right = controller.getRightTriggerAxis();
        double left = controller.getLeftTriggerAxis();
        double spud = 1;
        if (right > 0.005 && left < 0.005) {
            shoot.setSpeed(spud);
        } else if (left > 0.005 && right < 0.005) {
            shoot.setSpeed(-spud); 
        } else {
            shoot.stop();

        }
    }, shoot)
);
    arms.setDefaultCommand(
            Commands.run(() -> {
                double speed = controller.getLeftY();
                if (speed > 0.05 || speed < -0.05) {
                    arms.setSpeed(speed);
                } else {
                    arms.setSpeed(0);
                }
            }, arms)
    );

    drivetrain.registerTelemetry(logger::telemeterize);
}

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
