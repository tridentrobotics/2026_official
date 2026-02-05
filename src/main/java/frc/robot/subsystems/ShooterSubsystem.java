package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    // Declare the Talon SRX motor
    private final TalonSRX shooterMotor;
    
    // Constants - ADJUST THESE FOR YOUR ROBOT
    private static final int SHOOTER_MOTOR_ID = 30; // CAN ID of your Talon SRX
    private static final double SHOOTER_SPEED_PERCENT = 0.8; // 80% power (0.0 to 1.0)
    
    public ShooterSubsystem() {
        // Create the Talon SRX motor controller
        shooterMotor = new TalonSRX(SHOOTER_MOTOR_ID);
        
        // Configure the motor settings
        shooterMotor.setNeutralMode(NeutralMode.Coast); // Motor coasts when stopped
        shooterMotor.setInverted(false); // Change to true if motor spins backward
        
        // Factory reset to clear any previous configurations
        shooterMotor.configFactoryDefault();
    }
    
    /**
     * Spin up the shooter to the target speed
     */
    public void spinUp() {
        shooterMotor.set(ControlMode.PercentOutput, SHOOTER_SPEED_PERCENT);
    }
    
    /**
     * Stop the shooter
     */
    public void stop() {
        shooterMotor.set(ControlMode.PercentOutput, 0);
    }
    
    /**
     * Set shooter to a specific speed
     * @param speed speed as percent (0.0 to 1.0)
     */
    public void setSpeed(double speed) {
        shooterMotor.set(ControlMode.PercentOutput, speed);
    }
    
    /**
     * Get the current shooter motor output
     * @return motor output percent
     */
    public double getMotorOutput() {
        return shooterMotor.getMotorOutputPercent();
    }
    
    /**
     * Get the current draw of the motor
     * @return current in amps
     */
    public double getCurrent() {
        return shooterMotor.getStatorCurrent();
    }
    
    @Override
    public void periodic() {
        // This runs every 20ms - you can add telemetry here
        // SmartDashboard.putNumber("Shooter Speed", getMotorOutput());
    }
}