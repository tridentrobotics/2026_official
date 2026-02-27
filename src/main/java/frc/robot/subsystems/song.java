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
        
        var rick = m_orchestra.loadMusic("rick.chrp");


        if (!rick.isOK()) {
            System.out.println("Failed to load music: " + rick.toString());
        }
    }

        // Replace simple play with a reusable method call
        public void playsong() {
            // Delegate to the new method so callers can still use playsong()
            playSong("rick.chrp");
        }
    
        /**
         * Load and play the given .chrp file from the robot deploy directory.
         * Examples: "rick.chrp", "song2.chrp", "march.chrp"
         */
        public void playSong(String filename) {
            if (filename == null || filename.isEmpty()) {
                System.out.println("No filename provided to playSong()");
                return;
            }

            // Stop any currently playing music first
            var stopRes = m_orchestra.stop();
            if (!stopRes.isOK()) {
                System.out.println("Failed to stop previous song: " + stopRes.toString());
            }

            // Load the requested file
            var loadRes = m_orchestra.loadMusic(filename);
            if (!loadRes.isOK()) {
                System.out.println("Failed to load music '" + filename + "': " + loadRes.toString());
                return;
            }

            // Play the loaded file
            var playRes = m_orchestra.play();
            if (!playRes.isOK()) {
                System.out.println("Failed to play music '" + filename + "': " + playRes.toString());
            } else {
                System.out.println("Playing '" + filename + "'");
            }
        }
    
        public void stopsong() {
            var rick = m_orchestra.stop();
    
            if (!rick.isOK()) {
                System.out.println("Failed to stop music: " + rick.toString());
            }
        }
}

