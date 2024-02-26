package frc.robot.subsystems.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterTop extends SubsystemBase{
    // Motors
    private final CANSparkMax shooterTop = new CANSparkMax(12, MotorType.kBrushed);
    //private final CANSparkMax shooterBottom = new CANSparkMax(2, MotorType.kBrushed);
    //private final CANSparkMax feeder1= new CANSparkMax(3, MotorType.kBrushless);
    //private final CANSparkMax feeder2 = new CANSparkMax(4, MotorType.kBrushless);

    // Creates new Shooter
    public ShooterTop() {}

    // Sets 1st set of speeds
    public void set(double speed) {
        shooterTop.set(speed);
        //shooterBottom.set(speed);
        //feeder1.set(speed);
        //feeder2.set(speed);
    }

    
  @Override
  public void periodic() {
  }
}

