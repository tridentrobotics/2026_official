package frc.robot.subsystems;

// Imports go at the TOP, outside the class
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs.PCM;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Pneumatics extends SubsystemBase{
    
    private Compressor compressor;
    private Solenoid rightSolenoid;
    private Solenoid leftSolenoid;

    // Toggle tracking
    private boolean Extended = false;
    
    public Pneumatics() {
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();

        rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, PCM.rightSolenoidPort);
        leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, PCM.leftSolenoidPort);
    }
    public void extendSolenoids() {
        if (compressor.getPressureSwitchValue()) {
            Extended = true;
            rightSolenoid.set(Extended);
            leftSolenoid.set(Extended);
            System.out.println("Solenoids Extended");
        } else {
            System.err.println("Pressure to low, cannot extend solenoids");
        }
    }
    public void retractSolenoids() {
        Extended = false;
        rightSolenoid.set(Extended);
        leftSolenoid.set(Extended);
        System.out.println("Solenoids Retracted");
    }
    public boolean isPressureOK() {
        return compressor.getPressureSwitchValue();
    }
}

