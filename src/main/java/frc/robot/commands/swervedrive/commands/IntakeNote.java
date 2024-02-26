package frc.robot.commands.swervedrive.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.Intake;

public class IntakeNote extends Command{
    public final Intake intakeSubsystem;
    public final double speed;

    // Create new Feed.
    public IntakeNote(Intake intakeSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        

  // Use addRequirements() here to declare subsystem dependicies
  addRequirements(intakeSubsystem);
}
// Called when the command is intially scheduled
@Override
public void initialize() {}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    intakeSubsystem.set(speed);
    
}

// Called once the command ends or is interruped.
@Override
public void end(boolean interruped) {
    intakeSubsystem.set(0);
}

//Returns true when the command should end.
@Override
public boolean isFinished() {
    return false;
}
}


