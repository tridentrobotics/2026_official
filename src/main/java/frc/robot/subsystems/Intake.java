package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private final TalonFX intakeMotor = new TalonFX(Constants.CanIDs.IntakeMotor);

    /** The speed the operator requested (set by start/stop). */
    private double commandedSpeed = 0.0;

    /** Whether the intake is supposed to be running. */
    private boolean running = false;

    /** Jam-detection / unjam state machine. */
    private enum JamState { NORMAL, UNJAMMING }
    private JamState jamState = JamState.NORMAL;

    /** Timer used for both the startup grace period and the unjam duration. */
    private final Timer jamTimer = new Timer();

    /** 
     * Grace period (seconds) after start() before we begin checking for a jam.
     * This avoids a false-positive while the motor is still spinning up.
     */
    private static final double JAM_GRACE_PERIOD = 0.25;

    /** How long (seconds) to reverse the motor when a jam is detected. */
    private static final double UNJAM_DURATION = .1;

    /** Speed used when reversing to clear a jam. */
    private static final double UNJAM_SPEED = 0.5;

    /** Velocity threshold (rot/s) – at or below this the motor is considered stalled. */
    private static final double STALL_VELOCITY_THRESHOLD = 0.5;

    public Intake() {}

    public void start(double speed) {
        System.out.println("Intake started");
        commandedSpeed = speed;
        running = true;
        jamState = JamState.NORMAL;

        // Start the grace-period timer so we don't false-trigger on spin-up
        jamTimer.reset();
        jamTimer.start();

        intakeMotor.set(speed);
        double currentVelocity = intakeMotor.getVelocity().getValueAsDouble();
        System.out.println("intakespeed: " + currentVelocity);
    }

    public void stop() {
        intakeMotor.set(0.0);
        running = false;
        commandedSpeed = 0.0;
        jamState = JamState.NORMAL;
        jamTimer.stop();
        System.out.println("Intake stopped");
    }

    @Override
    public void periodic() {
        if (!running) return;

        double currentVelocity = intakeMotor.getVelocity().getValueAsDouble();

        switch (jamState) {
            case NORMAL:
                // Only check for a jam after the grace period has elapsed
                if (jamTimer.hasElapsed(JAM_GRACE_PERIOD)) {
                    if (Math.abs(currentVelocity) <= STALL_VELOCITY_THRESHOLD) {
                        // Motor is stuck – reverse to unjam
                        System.out.println("Intake jam detected! Reversing to unjam.");
                        double reverseDirection = -Math.signum(commandedSpeed);
                        intakeMotor.set(reverseDirection * UNJAM_SPEED);

                        jamTimer.reset();
                        jamTimer.start();
                        jamState = JamState.UNJAMMING;
                    }
                }
                break;

            case UNJAMMING:
                if (jamTimer.hasElapsed(UNJAM_DURATION)) {
                    // Unjam period over – go back to the original commanded speed
                    System.out.println("Unjam complete, resuming intake.");
                    intakeMotor.set(commandedSpeed);

                    jamTimer.reset();
                    jamTimer.start();
                    jamState = JamState.NORMAL;
                }
                break;
        }
    }
}