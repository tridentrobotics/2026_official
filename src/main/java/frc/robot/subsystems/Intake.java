package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
// Timer and unjam state machine removed — intake now has simple start/stop behavior
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ChangeLogger;

public class Intake extends SubsystemBase {

    private final TalonFX intakeMotor = new TalonFX(Constants.CanIDs.IntakeMotor);
    // Reusable change-logger
    private final ChangeLogger logger = new ChangeLogger("Intake");

    /** The speed the operator requested (set by start/stop). */
    private double commandedSpeed = 0.0;

    /** Whether the intake is supposed to be running. */
    // running flag removed — commandedSpeed determines motor output

    // Unjam/jam-detection removed. Intake will simply command the motor to the
    // requested speed while running.

    public Intake() {}

    

    public void start(double speed) {
    logger.logOnce("IntakeStarted", "Intake started");
    commandedSpeed = speed;
        intakeMotor.set(speed);
        double currentVelocity = intakeMotor.getVelocity().getValueAsDouble();
    logger.logOnce("Velocity", String.format("intakespeed: %.2f", currentVelocity));
    }

    public void stop() {
    intakeMotor.set(0.0);
        commandedSpeed = 0.0;
    logger.logOnce("IntakeStopped", "Intake stopped");
    }

    @Override
    public void periodic() {
        // Keep the motor at the commanded speed. If stopped, commandedSpeed will be 0.
        intakeMotor.set(commandedSpeed);
    }
}