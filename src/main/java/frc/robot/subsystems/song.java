package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.CanIDs.*;

public class song extends SubsystemBase {
    private final TalonFX SM1 = new TalonFX(shootMotor);
    private final TalonFX SM2 = new TalonFX(shootMotor2);
    private final TalonFX AM = new TalonFX(ArmMotorR);
    private final TalonFX AM2 = new TalonFX(ArmMotorL);
    private final TalonFX IM = new TalonFX(IntakeMotor);
    private final TalonFX FRD = new TalonFX(FRDrive);
    private final TalonFX FRS = new TalonFX(FRSteer);
    private final TalonFX FLD = new TalonFX(FLDrive);
    private final TalonFX FLS = new TalonFX(FLSteer);
    private final TalonFX BRD = new TalonFX(BRDrive);
    private final TalonFX BRS = new TalonFX(BRSteer);
    private final TalonFX BLD = new TalonFX(BLDrive);
    private final TalonFX BLS = new TalonFX(BLSteer);

    private final Orchestra m_orchestra = new Orchestra();

    private String currentSong = null; // Tracks the current song filename
    private boolean isPlaying = false;

    public song() {
        
        m_orchestra.addInstrument(BLD);
        m_orchestra.addInstrument(BRD);
        m_orchestra.addInstrument(FLD);
        m_orchestra.addInstrument(FRD);

        m_orchestra.addInstrument(FLS);
        m_orchestra.addInstrument(FRS);
        m_orchestra.addInstrument(BLS);
        m_orchestra.addInstrument(BRS);
        
        m_orchestra.addInstrument(SM1);
        m_orchestra.addInstrument(SM2);
        m_orchestra.addInstrument(AM);
        m_orchestra.addInstrument(AM2);
        m_orchestra.addInstrument(IM);
    }

    /** 
     * Plays a song. Stops the currently playing song first if one exists.
     */
    public void playSong(String filename) {
        if (isPlaying) {
            stopSong(); // Stop previous song
        }

        var loadResult = m_orchestra.loadMusic(filename);
        if (!loadResult.isOK()) {
            System.out.println("Failed to load music: " + loadResult.toString());
            return;
        }

        var playResult = m_orchestra.play();
        if (!playResult.isOK()) {
            System.out.println("Failed to play music: " + playResult.toString());
        } else {
            currentSong = filename;
            isPlaying = true;
            System.out.println("Playing song: " + filename);
        }
    }

    /**
     * Stops the current song
     */
    public void stopSong() {
        if (!isPlaying) return; // Nothing to stop

        var stopResult = m_orchestra.stop();
        if (!stopResult.isOK()) {
            System.out.println("Failed to stop music: " + stopResult.toString());
        } else {
            System.out.println("Stopped song: " + currentSong);
        }

        isPlaying = false;
        currentSong = null;
    }

    /** Returns whether a song is currently playing */
    public boolean isPlaying() {
        return isPlaying;
    }

    /** Returns the filename of the current song, or null if none */
    public String getCurrentSong() {
        return currentSong;
    }
}