package frc.robot.commands.swervedrive.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Elevator;


public class ElevatorUp extends Command{
    public final Elevator elevatorSubsystem;
    public final double speed;

    // Create new Stage Shooter.
    public ElevatorUp(Elevator elevatorSubsystem, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;

        // Use addRequirements() here to declare subsystem dependicies
        addRequirements(elevatorSubsystem);
    }
    
    // Called when the command is intially scheduled
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevatorSubsystem.set(-speed);
    }

    // Called once the command ends or is interruped.
    @Override
    public void end(boolean interruped) {
        elevatorSubsystem.set(0);
    }

    //Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}