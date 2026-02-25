package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private final TalonSRX intakeMotor = new TalonSRX(Constants.CanIDs.IntakeMotor);

    public Intake() {}

    public void start() {
        intakeMotor.set(ControlMode.PercentOutput, 0.2);
        System.out.println("Intake started");
    }

    public void stop() {
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
        System.out.println("Intake stopped");
    }
}