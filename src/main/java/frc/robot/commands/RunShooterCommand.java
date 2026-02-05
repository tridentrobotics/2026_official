package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterCommand extends Command {
    private final ShooterSubsystem shooter;
    
    public RunShooterCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        shooter.spinUp();
        System.out.println("Shooting");
    }
    
    @Override
    public void execute() {
        // Command keeps running
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}