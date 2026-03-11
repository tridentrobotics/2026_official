package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.lang.Math;

public class Shoot extends SubsystemBase {

    private final TalonFX shootMotor = new TalonFX(Constants.CanIDs.shootMotor);
    private final TalonFX shootMotor2 = new TalonFX(Constants.CanIDs.shootMotor2);

    public Shoot() {
        
    }

    public void setSpeed(double shooterSpeed) {
        double currentVelocity = shootMotor.getVelocity().getValueAsDouble();
        //double feedSpeed = (Math.abs(currentVelocity) < (90 * shooterSpeed) - 10) ? 0 : shooterSpeed;
        double feedSpeed = Math.abs(currentVelocity) < 90 ? 0 : 1;

        shootMotor.set(-shooterSpeed);
        shootMotor2.set(feedSpeed);
        if (shooterSpeed > 0) {
        System.out.println("Shoot speed: ," + shooterSpeed + "Feed Speed: ," + feedSpeed + "Velocity: " + currentVelocity);
        }

    }

    public void stop() {
        setSpeed(0.0);
    }
}