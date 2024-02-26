package frc.robot.subsystems.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Intake extends SubsystemBase{
    //motors
    // final CANSparkMax IntakeMovement = new CANSparkMax(14, MotorType.kBrushless);
    private final CANSparkMax TopSpinner = new CANSparkMax(15, MotorType.kBrushless);
    private final CANSparkMax BottomSpinner = new CANSparkMax(16, MotorType.kBrushless);
    
    //creates new Intake
    public Intake() {}

    public void set(double speed) {
    
        TopSpinner.set(-speed);
        BottomSpinner.set(speed);
    }
}
