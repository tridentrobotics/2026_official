package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shoot extends SubsystemBase {

    private final TalonSRX shootMotor = new TalonSRX(Constants.CanIDs.shootMotor);
    private final TalonSRX shootMotor2 = new TalonSRX(Constants.CanIDs.shootMotor2);

    public Shoot() {
        
    }

    public void setSpeed(double speed) {
        shootMotor.set(ControlMode.PercentOutput, speed);
        shootMotor2.set(ControlMode.PercentOutput, speed);
        if (speed > 0) {
        System.out.println("Shoot speed: " + speed);
        }
    }

    public void stop() {
        setSpeed(0.0);
    }
}