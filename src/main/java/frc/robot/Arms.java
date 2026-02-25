package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;


public class Arms extends SubsystemBase {
    private final TalonSRX armMotor = new TalonSRX(Constants.CanIDs.ArmMotor);
    private final XboxController joystick2 = new XboxController(1);

    public Arms() {
        double speed = joystick2.getLeftX();
        armMotor.set(ControlMode.PercentOutput, speed);
        System.out.println(speed + "Arm");
    }
    
    public void stop() {
        armMotor.set(ControlMode.PercentOutput,0.0);
    }

   
}
