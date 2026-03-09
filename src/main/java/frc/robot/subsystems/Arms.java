package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Arms extends SubsystemBase {

    private final TalonSRX rightMotor = new TalonSRX(Constants.CanIDs.ArmMotorR);
    private final TalonSRX leftMotor = new TalonSRX(Constants.CanIDs.ArmMotorL);

    private static final double TICKS_PER_ROTATION = 4096.0;

    private static final double RETRACTED_ROTATIONS = 0.0;
    private static final double EXTENDED_ROTATIONS = 0.35;

    private static final double ARM_SPEED = 0.325;
    private static final double POSITION_TOLERANCE = 0.005;

    // Gravity assist settings
    private static final boolean USE_GRAVITY_DROP = false;
    private static final double GRAVITY_DROP_POINT = 0.10;

    private boolean extended = false;
    private boolean moving = false;

    public Arms() {

        leftMotor.follow(rightMotor);

        // HARD stop on boot
        rightMotor.set(ControlMode.PercentOutput, 0);

        rightMotor.configSelectedFeedbackSensor(
            FeedbackDevice.CTRE_MagEncoder_Absolute,
            0,
            10
        );

        double absoluteTicks = rightMotor.getSelectedSensorPosition(0);
        rightMotor.setSelectedSensorPosition(absoluteTicks);

        // current limiting (anti-smoke insurance)
        rightMotor.configContinuousCurrentLimit(20);
        rightMotor.enableCurrentLimit(true);

        System.out.println("Arms initialized. Absolute ticks: " + absoluteTicks);
    }

    public void toggleArm() {
        extended = !extended;
        moving = true;
    }

    private double getCurrentRotations() {
        return rightMotor.getSelectedSensorPosition() / TICKS_PER_ROTATION;
    }

    private double getTargetRotations() {
        return extended ? EXTENDED_ROTATIONS : RETRACTED_ROTATIONS;
    }

    private void stopMotor() {
        rightMotor.set(ControlMode.PercentOutput, 0);
        moving = false;
    }

    @Override
    public void periodic() {

        if (!moving) {
            stopMotor();
            return;
        }

        double current = getCurrentRotations();
        double target = getTargetRotations();
        double error = target - current;

        // Gravity assist when lowering (optional)
        if (USE_GRAVITY_DROP && !extended && current <= GRAVITY_DROP_POINT) {
            stopMotor();
            return;
        }

        if (Math.abs(error) < POSITION_TOLERANCE) {
            stopMotor();
            return;
        }

        double speed = ARM_SPEED * Math.signum(error);
        rightMotor.set(ControlMode.PercentOutput, speed);

        System.out.printf(
            "Arm Rot: %.3f | Target: %.3f | Extended: %b | Moving: %b%n",
            current, target, extended, moving
        );
    }
}
