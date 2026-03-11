package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Arms extends SubsystemBase {

    private enum ArmState { IDLE, MOVING, GRAVITY_HOLDING }

    private final TalonSRX rightMotor = new TalonSRX(Constants.CanIDs.ArmMotorR);
    private final TalonSRX leftMotor = new TalonSRX(Constants.CanIDs.ArmMotorL);

    private static final double TICKS_PER_ROTATION = 4096.0;

    private static final double RETRACTED_ROTATIONS = 0.0;
    private static final double EXTENDED_ROTATIONS = 0.35;

    private static final double ARM_SPEED = 0.325;
    private static final double POSITION_TOLERANCE = 0.005;

    private double lastLoggedSpeed = Double.NaN;
    private double lastLoggedVelocity = Double.NaN;
    private double lastLoggedPosition = Double.NaN;

    // Gravity assist settings
    private static final boolean USE_GRAVITY_DROP = true;
    private static final double GRAVITY_DROP_POINT = 0.10;
    private static final double GRAVITY_DROP_RESISTANCE = -0.05;

    private boolean extended = false;
    private ArmState state = ArmState.IDLE;

    public Arms() {

        rightMotor.follow(leftMotor);

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

        System.out.println("Arms initialized. Absolute ticks: " + absoluteTicks);
    }

    /** Toggle arm between extended and retracted */
    public void toggleArm() {
        extended = !extended;
        state = ArmState.MOVING;
    }

    private double getCurrentRotations() {
        return rightMotor.getSelectedSensorPosition() / TICKS_PER_ROTATION;
    }

    private double getTargetRotations() {
        return extended ? EXTENDED_ROTATIONS : RETRACTED_ROTATIONS;
    }

    public void setSpeed(double speed) {    
        rightMotor.set(ControlMode.PercentOutput, speed);
        //leftMotor.set(ControlMode.PercentOutput, speed);
        if (speed != 0) {
        System.out.println("Arm speed: " + speed);
        }
    }


    private void stopMotor() {
        rightMotor.set(ControlMode.PercentOutput, 0);
        state = ArmState.IDLE;

        // Reset logging guards
        lastLoggedSpeed = Double.NaN;
        lastLoggedVelocity = Double.NaN;
    }

    @Override
    public void periodic() {

        if (state == ArmState.IDLE) return;

        double currentPos = getCurrentRotations();
        double target = getTargetRotations();
        double error = target - currentPos;
        double velocity = rightMotor.getSelectedSensorVelocity();

        if (state == ArmState.GRAVITY_HOLDING) {

            rightMotor.set(ControlMode.PercentOutput, GRAVITY_DROP_RESISTANCE);

            if (currentPos <= RETRACTED_ROTATIONS + POSITION_TOLERANCE) {
                stopMotor();
            }

            return;
        }

        // Transition to gravity hold when lowering
        if (USE_GRAVITY_DROP && !extended && currentPos <= GRAVITY_DROP_POINT) {

            System.out.printf(
                "Entering Gravity Hold | Pos: %.3f | Vel: %.0f%n",
                currentPos,
                velocity
            );

            state = ArmState.GRAVITY_HOLDING;
            return;
        }

        // Target reached
        if (Math.abs(error) < POSITION_TOLERANCE) {
            stopMotor();
            return;
        }

        double speed = ARM_SPEED * Math.signum(error);

        if (currentPos != lastLoggedPosition) {

            System.out.printf(
                "Arm Cmd: %.3f | Vel: %.0f | Pos: %.3f%n",
                speed,
                velocity,
                currentPos
            );
            lastLoggedSpeed = speed;
            lastLoggedVelocity = velocity;
            lastLoggedPosition = currentPos;
        }
        rightMotor.set(ControlMode.PercentOutput, speed);

    }
}
