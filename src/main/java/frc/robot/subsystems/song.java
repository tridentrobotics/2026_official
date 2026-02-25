package frc.robot;

// Imports go at the TOP, outside the class
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.TimedRobot;

public class song extends SubsystemBase{

    // Declare your motor and orchestra as class fields
    private final TalonFX motor1 = new TalonFX(61);
    private final TalonFX motor2 = new TalonFX(3); // device ID 0, change as needed
    private final Orchestra m_orchestra = new Orchestra();

    @Override
    public void robotInit() {
        m_orchestra.addInstrument(motor1);
        m_orchestra.addInstrument(motor2);

        var status = m_orchestra.loadMusic("output.chrp");

        if (!status.isOK()) {
            System.out.println("Failed to load music: " + status.toString());
        }
    }

    @Override
    public void teleopInit() {
        m_orchestra.play(); // Start playing when teleop begins!
    }

    @Override
    public void disabledInit() {
        m_orchestra.stop(); // Stop when disabled
    }
}

