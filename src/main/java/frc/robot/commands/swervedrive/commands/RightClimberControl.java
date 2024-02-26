/*package frc.robot.commands.swervedrive.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.RightClimber;


public class RightClimberControl extends Command{
    public final RightClimber rightClimberSubsystem;
    public final double speed;

    // Create new Stage Shooter.
    public RightClimberControl(RightClimber rightClimberSubsystem, double speed) {
        this.rightClimberSubsystem = rightClimberSubsystem;
        this.speed = speed;

        // Use addRequirements() here to declare subsystem dependicies
        addRequirements(rightClimberSubsystem);
    }
    
    // Called when the command is intially scheduled
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        rightClimberSubsystem.set(-speed);
    }

    // Called once the command ends or is interruped.
    @Override
    public void end(boolean interruped) {
        rightClimberSubsystem.set(0);
    }

    //Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}*/