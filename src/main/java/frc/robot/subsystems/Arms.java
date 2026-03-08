package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arms extends SubsystemBase {

    private final TalonSRX armMotorR = new TalonSRX(Constants.CanIDs.ArmMotorR);
    private final TalonSRX armMotorL = new TalonSRX(Constants.CanIDs.ArmMotorL);

    private static final double TICKS_PER_ROTATION = 4096.0;
    private static final double TARGET_ROTATIONS = 0.35; // toggle target

    private boolean extended = false;  // toggle state
    private boolean moving = false;    // motor is moving

    public Arms() {
        armMotorL.follow(armMotorR);
        armMotorR.set(ControlMode.PercentOutput, 0.0);

        // Absolute encoder
        armMotorR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);

        // Sync relative encoder to absolute at startup
        double absTicks = armMotorR.getSelectedSensorPosition(0);
        armMotorR.setSelectedSensorPosition(absTicks);

        System.out.println("Arm initialized. Absolute ticks: " + absTicks);
    }

    /** Toggle between 0 and 0.35 rotations */
    public void toggleArm() {
        extended = !extended;
        moving = true;
    }

    @Override
    public void periodic() {
        double currentRot = armMotorR.getSelectedSensorPosition() / TICKS_PER_ROTATION;

        if (moving) {
            double target = extended ? TARGET_ROTATIONS : 0.0;
            double error = target - currentRot;

            // Only move if not at target
            if (Math.abs(error) > 0.005) { // small tolerance
                double speed = 0.325 * Math.signum(error); // ±0.325 toward target
                armMotorR.set(ControlMode.PercentOutput, speed);
            } else {
                armMotorR.set(ControlMode.PercentOutput, 0);
                moving = false; // reached target
            }
        } else {
            armMotorR.set(ControlMode.PercentOutput, 0);
        }

        // Logging
        System.out.printf(
            "Arm Rot: %.3f | Target: %.3f | Extended: %b | Moving: %b%n",
            currentRot, extended ? TARGET_ROTATIONS : 0.0, extended, moving
        );
    }
}