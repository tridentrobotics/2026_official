package frc.robot.subsystems;

//import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arms extends SubsystemBase {
    private final TalonFX armMotorR = new TalonFX(Constants.CanIDs.ArmMotorR);
    private final TalonFX armMotorL = new TalonFX(Constants.CanIDs.ArmMotorL);
    public Arms() {
        // Subsystem should not read controllers in its constructor.
        // Leave motors idle until commands call setSpeed/stop.
        armMotorR.set(0.0);
        armMotorL.set(0.0);
    }

    
    public void setSpeed(double speed) {
        if (speed > .05 || speed < -.05) {
        armMotorR.set(speed);
        armMotorL.set(speed);
        System.out.println("Arm speed: " + speed);
        }
    }
    

    public void stop() {
        setSpeed(0.0);
    }

   
}
