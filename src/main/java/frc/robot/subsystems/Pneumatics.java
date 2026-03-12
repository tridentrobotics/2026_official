package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs.PCM;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.util.ChangeLogger;

public class Pneumatics extends SubsystemBase {

    private Compressor compressor;
    private DoubleSolenoid rightSolenoid;
    private DoubleSolenoid leftSolenoid;

    // Toggle tracking
    private boolean isExtended = false;

    public Pneumatics() {
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        compressor.enableDigital();

        rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PCM.rightSolenoidPort1, PCM.rightSolenoidPort2);
        leftSolenoid  = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PCM.leftSolenoidPort1,  PCM.leftSolenoidPort2);
    }

    private final ChangeLogger logger = new ChangeLogger("Pneumatics");

    /** Toggles solenoids between extended and retracted. */
    public void toggleSolenoids() {
        if (!isExtended && !compressor.getPressureSwitchValue()) {
            logger.logOnce("PressureError", "Pressure too low, cannot extend solenoids");
            return;
        }

        isExtended = !isExtended;
        Value position = isExtended ? Value.kForward : Value.kReverse;

        rightSolenoid.set(position);
        leftSolenoid.set(position);

        logger.logOnce("Solenoids", "Solenoids " + (isExtended ? "Extended" : "Retracted"));
    }

    public boolean isExtended() {
        return isExtended;
    }

    public boolean isPressureOK() {
        return compressor.getPressureSwitchValue();
    }
}