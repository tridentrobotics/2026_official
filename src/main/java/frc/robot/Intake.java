package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Intake{
    private TalonSRX intakeMotor = new TalonSRX(30);
    private CommandXboxController drivercontroller= new CommandXboxController(0);
    private boolean isrunning = false;
    
    public Intake(){
configureBindings();
    }
    private void configureBindings(){
drivercontroller.a().onTrue(new InstantCommand(()-> toggleIntake()));

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
intakeMotor.set(ControlMode.PercentOutput, 1);
System.out.println("Intake Started");
}
private void stopIntake(){
intakeMotor.set(ControlMode.PercentOutput, 0);
System.out.println("Intake Stopped");
}
}

