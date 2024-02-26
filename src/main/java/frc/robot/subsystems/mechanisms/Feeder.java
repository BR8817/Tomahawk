package frc.robot.subsystems.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase{
    // Motors
    private final CANSparkMax feeder1= new CANSparkMax(10, MotorType.kBrushless);
    private final CANSparkMax feeder2 = new CANSparkMax(11, MotorType.kBrushless);

    // Creates new Shooter
    public Feeder() {}

    // Sets 1st set of speeds
    public void set(double speed) {
        feeder1.set(speed);
        feeder2.set(speed);
    }

    
  @Override
  public void periodic() {
  }
}