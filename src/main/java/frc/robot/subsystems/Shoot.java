package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.ChangeLogger;

public class Shoot extends SubsystemBase {

    private final TalonFX shootMotor = new TalonFX(Constants.CanIDs.shootMotor);
    private final TalonFX shootMotor2 = new TalonFX(Constants.CanIDs.shootMotor2);

    private final ChangeLogger logger = new ChangeLogger("Shoot");

    // With velocity filter removed, feeding will start immediately when shooter is commanded.

    public Shoot() {}

    public void setSpeed(double shooterSpeed) {
        // Negative input => immediate both motors spin (bypass spin-up)
        if (shooterSpeed < 0.0) {
            double power = -shooterSpeed;
            shootMotor.set(power);
            return;
        }

        // Zero -> stop everything and reset feed state
        if (shooterSpeed == 0.0) {
            shootMotor.set(0.0);
            // No velocity filter — nothing to reset
            return;
        }

        // Positive speeds: immediately run the shooter and enable feeding without velocity filtering
        shootMotor.set(-shooterSpeed);

    }

    public void feed() {
        shootMotor2.set(0.2);
    }

    /**
     * Stop the feed motor only (leaves shooter wheel state unchanged).
     */
    public void stopFeed() {
        shootMotor2.set(0.0);
    }

    public void stop() {
        setSpeed(0.0);
    }
}