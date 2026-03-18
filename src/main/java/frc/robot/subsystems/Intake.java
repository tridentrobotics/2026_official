package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
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
    private boolean running = false;

    /** Jam-detection / unjam state machine. */
    // NORMAL: regular operation
    // STOP1: stop for a short period after detecting a jam
    // REVERSE: spin in the reverse direction to clear the jam
    // STOP2: stop again briefly before resuming forward
    private enum JamState { NORMAL, STOP1, REVERSE, STOP2 }
    private JamState jamState = JamState.NORMAL;

    /** Timer used for both the startup grace period and the unjam duration. */
    private final Timer jamTimer = new Timer();

    /** 
     * Grace period (seconds) after start() before we begin checking for a jam.
     * This avoids a false-positive while the motor is still spinning up.
     */
    private static final double JAM_GRACE_PERIOD = 0.25;

    /** How long (seconds) to stop between stages. */
    private static final double STOP_DURATION = .5;

    /** How long (seconds) to reverse the motor when a jam is detected. */
    private static final double REVERSE_DURATION = .1;

    /** Speed used when reversing to clear a jam (fractional, 0..1). */
    private static final double UNJAM_REVERSE_SPEED = 0.1;

    /** Velocity threshold (rot/s) – at or below this the motor is considered stalled. */
    private static final double STALL_VELOCITY_THRESHOLD = 0.5;

    public Intake() {}

    public void start(double speed) {
    logger.logOnce("IntakeStarted", "Intake started");
        commandedSpeed = speed;
        running = true;
        jamState = JamState.NORMAL;

        // Start the grace-period timer so we don't false-trigger on spin-up
        jamTimer.reset();
        jamTimer.start();

        intakeMotor.set(speed);
        double currentVelocity = intakeMotor.getVelocity().getValueAsDouble();
    logger.logOnce("Velocity", String.format("intakespeed: %.2f", currentVelocity));
    }

    public void stop() {
        intakeMotor.set(0.0);
        running = false;
        commandedSpeed = 0.0;
        jamState = JamState.NORMAL;
        jamTimer.stop();
    logger.logOnce("IntakeStopped", "Intake stopped");
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
                        // Motor is stuck – start multi-step unjam sequence:
                        // stop 1s -> reverse 1s -> stop 1s -> resume forward
                            logger.logOnce("JamDetected", "Intake jam detected! Starting unjam sequence: stop -> reverse -> stop -> forward.");

                        // Stop immediately
                        intakeMotor.set(0.0);

                        jamTimer.reset();
                        jamTimer.start();
                        jamState = JamState.STOP1;
                    }
                }
                break;
            case STOP1:
                if (jamTimer.hasElapsed(STOP_DURATION)) {
                    // Begin reversing to try to clear the jam
                    double reverseDirection = -Math.signum(commandedSpeed);
                    // If commandedSpeed was 0, default to -1 (reverse) to attempt clearing
                    if (reverseDirection == 0.0) reverseDirection = -1.0;
                    intakeMotor.set(reverseDirection * UNJAM_REVERSE_SPEED);

                    logger.logOnce("JamAction", String.format("Unjam: reversing for %.2fs.", REVERSE_DURATION));

                    jamTimer.reset();
                    jamTimer.start();
                    jamState = JamState.REVERSE;
                }
                break;

            case REVERSE:
                if (jamTimer.hasElapsed(REVERSE_DURATION)) {
                    // Stop again before resuming forward
                    intakeMotor.set(0.0);
                    logger.logOnce("JamAction", "Unjam: stopping briefly before resuming forward.");

                    jamTimer.reset();
                    jamTimer.start();
                    jamState = JamState.STOP2;
                }
                break;

            case STOP2:
                if (jamTimer.hasElapsed(STOP_DURATION)) {
                    // Sequence complete — resume the originally commanded speed
                    logger.logOnce("JamAction", "Unjam complete, resuming intake.");
                    intakeMotor.set(commandedSpeed);

                    jamTimer.reset();
                    jamTimer.start();
                    jamState = JamState.NORMAL;
                }
        break;
        }
    }
}