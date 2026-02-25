package frc.robot;

import edu.wpi.first.math.MathUtil;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shoot extends SubsystemBase{
    private TalonSRX shootMotor = new TalonSRX(Constants.CanIDs.shootMotor);
    private TalonSRX shootMotor2 = new TalonSRX(Constants.CanIDs.shootMotor2);
    private XboxController joystick = new XboxController(1);
    private double speed = 0;
    
    public Shoot(){
    speed = MathUtil.applyDeadband(joystick.getLeftX(), 0.05);
    shootMotor.set(ControlMode.PercentOutput, speed);
    shootMotor2.set(ControlMode.PercentOutput, speed);
    System.out.println(speed + "Shoot");
    }
}

