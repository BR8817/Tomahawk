package frc.robot.subsystems.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    // Motors
    private final CANSparkMax Elevator = new CANSparkMax(9, MotorType.kBrushless);
    

    // Creates new Shooter
    public Elevator() {}

    // Sets 1st set of speeds
    public void set(double speed) {
        Elevator.set(speed);
        
    }

    
  @Override
  public void periodic() {
  }
}
