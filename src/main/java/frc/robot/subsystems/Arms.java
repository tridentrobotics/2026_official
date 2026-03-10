package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Arms extends SubsystemBase {

    private enum ArmState { IDLE, MOVING }

    private final TalonSRX rightMotor = new TalonSRX(Constants.CanIDs.ArmMotorR);
    private final TalonSRX leftMotor = new TalonSRX(Constants.CanIDs.ArmMotorL);

    private static final double TICKS_PER_ROTATION = 4096.0;

    private static final double RETRACTED_ROTATIONS = 0.0;
    private static final double EXTENDED_ROTATIONS = 0.35;

    private static final double ARM_SPEED = 0.325;
    private static final double POSITION_TOLERANCE = 0.005;

    private double lastLoggedSpeed = Double.NaN;

    // Gravity assist settings
    private static final boolean USE_GRAVITY_DROP = true;
    private static final double GRAVITY_DROP_POINT = 0.10;
    private static final double GRAVITY_DROP_RESISTANCE = -0.05;

    private boolean extended = false;
    private ArmState state = ArmState.IDLE;

    public Arms() {

        leftMotor.follow(rightMotor);

        rightMotor.setNeutralMode(NeutralMode.Brake);
        leftMotor.setNeutralMode(NeutralMode.Brake);

        rightMotor.set(ControlMode.PercentOutput, 0);

        rightMotor.configSelectedFeedbackSensor(
            FeedbackDevice.CTRE_MagEncoder_Absolute,
            0,
            10
        );

        double absoluteTicks = rightMotor.getSelectedSensorPosition(0);
        rightMotor.setSelectedSensorPosition(absoluteTicks);

        rightMotor.configContinuousCurrentLimit(20);
        rightMotor.configPeakCurrentLimit(30);
        rightMotor.configPeakCurrentDuration(100);
        rightMotor.enableCurrentLimit(true);

        rightMotor.configOpenloopRamp(0.15);

        // PID gains
        rightMotor.config_kP(0, 6.0);
        rightMotor.config_kI(0, 0.0);
        rightMotor.config_kD(0, 80.0);
        rightMotor.config_kF(0, 0.0);

        System.out.println("Arms initialized. Absolute ticks: " + absoluteTicks);
    }

    /** Toggle arm between extended and retracted */
    public void toggleArm() {
        extended = !extended;
        state = ArmState.MOVING;
    }

    private double rotationsToTicks(double rotations) {
        return rotations * TICKS_PER_ROTATION;
    }

    private double getCurrentRotations() {
        return rightMotor.getSelectedSensorPosition() / TICKS_PER_ROTATION;
    }

    private double getTargetRotations() {
        return extended ? EXTENDED_ROTATIONS : RETRACTED_ROTATIONS;
    }

    private void stopMotor() {
        rightMotor.set(ControlMode.PercentOutput, 0);
        state = ArmState.IDLE;
        lastLoggedSpeed = Double.NaN;
    }

    @Override
    public void periodic() {

        if (state == ArmState.IDLE) return;

        double current = getCurrentRotations();
        double target = getTargetRotations();
        double error = target - current;
        double velocity = rightMotor.getSelectedSensorVelocity();

        // Stop if we are close enough to target
        if (Math.abs(error) < POSITION_TOLERANCE) {
            stopMotor();
            return;
        }

        // Feedforward to assist gravity when lowering past the drop point
        double feedforward = 0.0;
        if (USE_GRAVITY_DROP && !extended && current <= GRAVITY_DROP_POINT) {
            feedforward = GRAVITY_DROP_RESISTANCE;
        }


        // PID position control with feedforward
        rightMotor.set(
            ControlMode.Position,
            rotationsToTicks(target),
            DemandType.ArbitraryFeedForward,
            feedforward
        );
    }
}
