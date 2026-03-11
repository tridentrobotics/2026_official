package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private final TalonFX intakeMotor = new TalonFX(Constants.CanIDs.IntakeMotor);

    public Intake() {}

    public void start(double speed) {
        intakeMotor.set(speed);
        System.out.println("Intake started");
    }

    public void stop() {
        intakeMotor.set(0.0);
        System.out.println("Intake stopped");
    }
}