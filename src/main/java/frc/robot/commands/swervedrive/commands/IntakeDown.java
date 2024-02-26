package frc.robot.commands.swervedrive.commands;


import frc.robot.subsystems.mechanisms.IntakeArm;
import edu.wpi.first.wpilibj2.command.Command;


public class IntakeDown extends Command{
    public final IntakeArm intakeArmSubsystem;
    public final double speed;
    
    //Create new Intake
public IntakeDown(IntakeArm intakeArmSubsystem, double speed) {
    this.intakeArmSubsystem = intakeArmSubsystem;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependicies
    addRequirements(intakeArmSubsystem);
}
// Called when the command is intially scheduled
@Override
public void initialize() {}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    intakeArmSubsystem.set(-speed);
}

// Called once the command ends or is interruped.
@Override
public void end(boolean interruped) {
    intakeArmSubsystem.set(0);
}

//Returns true when the command should end.
@Override
public boolean isFinished() {
    return false;
    }
}
