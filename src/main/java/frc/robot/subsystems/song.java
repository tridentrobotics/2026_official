package frc.robot.subsystems;

// Imports go at the TOP, outside the class
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CanIDs.*;

public class song extends SubsystemBase{

    // Declare your motor and orchestra as class fields
    private final TalonFX motor1 = new TalonFX(shootMotor);
    private final TalonFX motor2 = new TalonFX(shootMotor2); // device ID 0, change as needed
    private final TalonFX motor3 = new TalonFX(ArmMotor);
    private final TalonFX motor4 = new TalonFX(IntakeMotor);
    private final Orchestra m_orchestra = new Orchestra();

    public song() {
        m_orchestra.addInstrument(motor1);
        m_orchestra.addInstrument(motor2);
        m_orchestra.addInstrument(motor3);
        m_orchestra.addInstrument(motor4);

        var status = m_orchestra.loadMusic("output.chrp");

        if (!status.isOK()) {
            System.out.println("Failed to load music: " + status.toString());
        }
    }

        public void playsong() {
            var status = m_orchestra.play();
    
            if (!status.isOK()) {
                System.out.println("Failed to play music: " + status.toString());
            }
        }
    
        public void stopsong() {
            var status = m_orchestra.stop();
    
            if (!status.isOK()) {
                System.out.println("Failed to stop music: " + status.toString());
            }
        }
}

