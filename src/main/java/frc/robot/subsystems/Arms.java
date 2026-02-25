package frc.robot.subsystems;

//import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arms extends SubsystemBase {
    private final TalonSRX armMotor = new TalonSRX(Constants.CanIDs.ArmMotor);
    public Arms() {
        // Subsystem should not read controllers in its constructor.
        // Leave motors idle until commands call setSpeed/stop.
        armMotor.set(ControlMode.PercentOutput, 0.0);
    }

    
    public void setSpeed(double speed) {
        armMotor.set(ControlMode.PercentOutput, speed);
        if (speed > 0) {
        System.out.println("Arm speed: " + speed);
        }
    }

    public void stop() {
        setSpeed(0.0);
    }

   
}
