package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Intake extends SubsystemBase {
    private TalonSRX IntakeMotor = new TalonSRX(Constants.CanIDs.IntakeMotor);
    private XboxController joystick = new XboxController(1);
    private boolean isrunning = false;
    
    public Intake(){
if (joystick.getAButtonPressed()){
    toggleIntake();
}
    }
    private void toggleIntake(){
        if(isrunning){
            stopIntake();
        }else{
            startIntake();
        }
        isrunning=!isrunning;
    }
private void startIntake(){
IntakeMotor.set(ControlMode.PercentOutput, 1);
System.out.println("Intake Started");
}
private void stopIntake(){
IntakeMotor.set(ControlMode.PercentOutput, 0);
System.out.println("Intake Stopped");
}
}

