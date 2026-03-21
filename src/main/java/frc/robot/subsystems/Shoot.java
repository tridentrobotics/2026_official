package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ChangeLogger;

public class Shoot extends SubsystemBase {

    private final TalonFX shootMotor = new TalonFX(Constants.CanIDs.shootMotor);
    private final TalonFX shootMotor2 = new TalonFX(Constants.CanIDs.shootMotor2);

    private final ChangeLogger logger = new ChangeLogger("Shoot");

    private static final double FEED_VEL_THRESHOLD = 90.0;
    private static final double FEED_DELAY_SECONDS = 0.3;

    private final edu.wpi.first.wpilibj.Timer feedDelayTimer = new edu.wpi.first.wpilibj.Timer();
    private boolean waitingForFeed = false;
    private boolean feedEnabled = false;

    public Shoot() {}

    public void setSpeed(double shooterSpeed) {
        // Negative input => immediate both motors spin (bypass spin-up)
        if (shooterSpeed < 0.0) {
            double power = -shooterSpeed;
            shootMotor.set(power);
            shootMotor2.set(power);
            feedDelayTimer.stop();
            feedDelayTimer.reset();
            waitingForFeed = false;
            feedEnabled = true; // keep feed active while override is applied
            logger.logOnce("FeedImmediate", "Negative speed input — feeding immediately");
            return;
        }

        // Zero -> stop everything and reset feed state
        if (shooterSpeed == 0.0) {
            shootMotor.set(0.0);
            shootMotor2.set(0.0);
            feedDelayTimer.stop();
            feedDelayTimer.reset();
            waitingForFeed = false;
            feedEnabled = false;
            return;
        }

        // Positive speeds: normal spin-up logic
        shootMotor.set(-shooterSpeed);

        double currentVelocity = Math.abs(shootMotor.getVelocity().getValueAsDouble());

        if (!feedEnabled) {
            if (currentVelocity >= FEED_VEL_THRESHOLD) {
                // Velocity is good — start the timer if we haven't yet
                if (!waitingForFeed) {
                    feedDelayTimer.reset();
                    feedDelayTimer.start();
                    waitingForFeed = true;
                    logger.logOnce("FeedTimerStart", "Shooter at speed, waiting for feed delay...");
                }
                // Check if delay has elapsed
                if (feedDelayTimer.hasElapsed(FEED_DELAY_SECONDS)) {
                    feedEnabled = true;
                    logger.logOnce("FeedEnabled", "Feed motor enabled after delay");
                }
            } else {
                // Dropped below threshold before timer finished — restart
                feedDelayTimer.stop();
                feedDelayTimer.reset();
                waitingForFeed = false;
            }
        }

        shootMotor2.set(feedEnabled ? 0.1 : 0.0);
    }

    public void stop() {
        setSpeed(0.0);
    }
}