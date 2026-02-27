package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CanIDs.*;

public class song extends SubsystemBase {

    private final TalonFX motor1 = new TalonFX(shootMotor);
    private final TalonFX motor2 = new TalonFX(shootMotor2);
    private final TalonFX motor3 = new TalonFX(ArmMotor);
    private final TalonFX motor4 = new TalonFX(IntakeMotor);

    private final Orchestra m_orchestra = new Orchestra();

    public song() {
        m_orchestra.addInstrument(motor1);
        m_orchestra.addInstrument(motor2);
        m_orchestra.addInstrument(motor3);
        m_orchestra.addInstrument(motor4);
    }

    // NEW METHOD: Load and play a song from filename
    public void playSong(String filename) {

        var loadResult = m_orchestra.loadMusic(filename);

        if (!loadResult.isOK()) {
            System.out.println("Failed to load music: " + loadResult.toString());
            return;
        }

        var playResult = m_orchestra.play();

        if (!playResult.isOK()) {
            System.out.println("Failed to play music: " + playResult.toString());
        } else {
            System.out.println("Playing song: " + filename);
        }
    }

    public void stopSong() {
        var stopResult = m_orchestra.stop();

        if (!stopResult.isOK()) {
            System.out.println("Failed to stop music: " + stopResult.toString());
        }
    }
}