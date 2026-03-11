package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shoot extends SubsystemBase {

    private final TalonFX shootMotor = new TalonFX(Constants.CanIDs.shootMotor);
    private final TalonFX shootMotor2 = new TalonFX(Constants.CanIDs.shootMotor2);

    public Shoot() {
        
    }

    public void setSpeed(double speed) {
        shootMotor.set(-speed);
        shootMotor2.set(speed);
        if (speed > 0) {
        System.out.println("Shoot speed: " + speed);
        }
    }

    public void stop() {
        setSpeed(0.0);
    }
}