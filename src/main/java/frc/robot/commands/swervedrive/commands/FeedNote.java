package frc.robot.commands.swervedrive.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Feeder;



public class FeedNote extends Command{
    public final Feeder feederSubsystem;
    public final double speed;
    
    // Create new Feed.
    public FeedNote(Feeder feederSubsystem, double speed) {
        this.feederSubsystem = feederSubsystem;
        this.speed = speed;
        

  // Use addRequirements() here to declare subsystem dependicies
  addRequirements(feederSubsystem);
}
// Called when the command is intially scheduled
@Override
public void initialize() {}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    feederSubsystem.set(speed);
    
}

// Called once the command ends or is interruped.
@Override
public void end(boolean interruped) {
    feederSubsystem.set(0);
}

//Returns true when the command should end.
@Override
public boolean isFinished() {
    return false;
}
}

