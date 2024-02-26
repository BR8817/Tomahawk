/*package frc.robot.commands.swervedrive.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanisms.LeftClimber;


public class LeftClimberControl extends Command{
    public final LeftClimber leftClimberSubsystem;
    public final double speed;

    // Create new Stage Shooter.
    public LeftClimberControl(LeftClimber leftClimberSubsystem, double speed) {
        this.leftClimberSubsystem = leftClimberSubsystem;
        this.speed = speed;

        // Use addRequirements() here to declare subsystem dependicies
        addRequirements(leftClimberSubsystem);
    }
    
    // Called when the command is intially scheduled
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        leftClimberSubsystem.set(-speed);
    }

    // Called once the command ends or is interruped.
    @Override
    public void end(boolean interruped) {
        leftClimberSubsystem.set(0);
    }

    //Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}*/