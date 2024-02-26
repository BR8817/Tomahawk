package frc.robot.subsystems.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterBottom extends SubsystemBase{
    // Motors
    //private final CANSparkMax shooterTop = new CANSparkMax(1, MotorType.kBrushed);
    private final CANSparkMax shooterBottom = new CANSparkMax(13, MotorType.kBrushed);
    //private final CANSparkMax feeder1= new CANSparkMax(3, MotorType.kBrushless);
    //private final CANSparkMax feeder2 = new CANSparkMax(4, MotorType.kBrushless);

    // Creates new Shooter
    public ShooterBottom() {}

    // Sets 1st set of speeds
    public void set(double speed) {
        shooterBottom.set(speed);
       
    }

    
  @Override
  public void periodic() {
  }
}

