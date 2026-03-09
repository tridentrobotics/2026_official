// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.song;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
    public static Joystick joystick = new Joystick(0);
    public static CommandXboxController controller = new CommandXboxController(1);

    public final song m_song = new song();
    private final Pneumatics Pneumatics = new Pneumatics();

    private double MaxSpeed;
    private double MaxAngularRate = RotationsPerSecond.of(0.4).in(RadiansPerSecond);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Arms arms;
    public final Shoot shoot;
    public final Intake intake;

    public RobotContainer() {
        System.out.println(alliance.Alliance); // your majesty there is a second bus coming!!!!!!!!!!!!1 yayayayayyayyaayya it sfun to stay at the commmraaaadd yayaya you should sleep on the ground i love touching your balls and then when we will touch more minors i love touchng qall the comrads i lopve touching minors and i will touch more minors and then I wil touch all your minors with an "e"
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
        final var idle = new SwerveRequest.Idle();

        autoChooser.setDefaultOption("Blue 1", Commands.sequence(
                autoFactory.resetOdometry("Blue_Top_1"),
                autoFactory.trajectoryCmd("Blue_Top_1"),
                shootSequence(),
                autoFactory.trajectoryCmd("Blue_Top_2"),
                drivetrain.applyRequest(() -> idle)
        ));

        autoChooser.addOption("Blue Mid", Commands.sequence(
                autoFactory.resetOdometry("Blue_Mid_1"),
                autoFactory.trajectoryCmd("Blue_Mid_1"),
                shootSequence(),
                autoFactory.trajectoryCmd("Blue_Mid_2"),
                drivetrain.applyRequest(() -> idle)
        ));

        autoChooser.addOption("Blue Bot", Commands.sequence(
                autoFactory.resetOdometry("Blue_Bot_1"),
                autoFactory.trajectoryCmd("Blue_Bot_1"),
                shootSequence(),
                autoFactory.trajectoryCmd("Blue_Bot_2"),
                drivetrain.applyRequest(() -> idle)
        ));

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

        controller.b().toggleOnTrue(
                Commands.startEnd(
                        () -> m_song.playSong("LoZMain.chrp"),  
                        () -> m_song.stopSong(),
                        m_song
                )
        );
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
        
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {

                    MaxSpeed = Math.abs(joystick.getRawAxis(5))
                            * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

                    return drive
                            .withVelocityX(joystick.getY() * MaxSpeed)
                            .withVelocityY(joystick.getX() * MaxSpeed)
                            .withRotationalRate(joystick.getZ() * MaxAngularRate);
                }));

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        new JoystickButton(joystick, 6).whileTrue(drivetrain.applyRequest(() -> brake));

        new JoystickButton(joystick, 4).whileTrue(
                drivetrain.applyRequest(() ->
                        point.withModuleDirection(new Rotation2d(-joystick.getY(), -joystick.getX()))
                ));

        new JoystickButton(joystick, 8)
                .and(new JoystickButton(joystick, 9))
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

        new JoystickButton(joystick, 8)
                .and(new JoystickButton(joystick, 7))
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        new JoystickButton(joystick, 2)
                .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Intake toggle
        controller.x().toggleOnTrue(
                Commands.startEnd(
                        () -> intake.start(),
                        () -> intake.stop(),
                        intake
                )
        );

        // Shooter default command
        shoot.setDefaultCommand(
                Commands.run(() -> {

                    double speed = joystick.getRawAxis(7);

                    if (joystick.getRawButton(18)) {
                        if (speed != -1) {
                            shoot.setSpeed(speed);
                        } else {
                            shoot.setSpeed(1.0);
                        }
                    } else {
                        shoot.stop();
                    }

                }, shoot)
        );

        // Arms default command

        //controller.leftBumper().onTrue(
        //Commands.runOnce(() -> {
        //        arms.toggleArm();
        //})
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Arms getArms() { return arms; }
    public Shoot getShoot() { return shoot; }
    public Intake getIntake() { return intake; }
}
