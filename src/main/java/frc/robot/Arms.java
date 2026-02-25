package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Arms subsystem: controls a single TalonFX-based arm motor.
 *
 * This class supports optional forward/reverse limit switches. If a
 * limit switch channel is set to -1 in Constants, the switch is
 * treated as absent and the subsystem will not attempt to read it.
 */
public class Arms extends SubsystemBase {
    private final TalonSRX armMotor = new TalonSRX(Constants.ArmMotor);
    private final XboxController joystick2 = new XboxController(1);

    // Optional limit switch inputs (may be null if not configured)
    private final DigitalInput forwardLimit;
    private final DigitalInput reverseLimit;

    public Arms() {
        // Construct optional DigitalInput instances only when configured.
        forwardLimit = (Constants.ArmForwardLimit >= 0) ? new DigitalInput(Constants.ArmForwardLimit) : null;
        reverseLimit = (Constants.ArmReverseLimit >= 0) ? new DigitalInput(Constants.ArmReverseLimit) : null;

        // Default teleop behavior: drive arm with left X axis of controller on port 1
        double speed = joystick2.getLeftX();
        setDefaultCommand(new RunCommand(() -> setSpeed(joystick2.getLeftX()), this));
    }

    /** Set raw motor output for the arm, respecting configured limits. */
    public void setSpeed(double speed) {
        double applied = speed;

        // If forward limit exists and is triggered, prevent motion in the forward direction
        if (speed > 0 && isForwardLimitTriggered()) {
            applied = 0.0;
        }

        // If reverse limit exists and is triggered, prevent motion in the reverse direction
        if (speed < 0 && isReverseLimitTriggered()) {
            applied = 0.0;
        }



        
        System.out.println(speed);
        
    }

    /** Stop the arm motor. */
    public void stop() {
        armMotor.set(ControlMode.PercentOutput,0.0);
    }

    /**
     * Returns true if the forward (upper) limit switch is present and pressed.
     */
    public boolean isForwardLimitTriggered() {
        return forwardLimit != null && !forwardLimit.get();
    }

    /**
     * Returns true if the reverse (lower) limit switch is present and pressed.
     */
    public boolean isReverseLimitTriggered() {
        return reverseLimit != null && !reverseLimit.get();
    }
}
