package frc.robot.subsystems.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class IntakeArm extends SubsystemBase{
    //motors
    // final CANSparkMax IntakeMovement = new CANSparkMax(14, MotorType.kBrushless);

    private final CANSparkMax intakeMover = new CANSparkMax(14, MotorType.kBrushed);
    //creates new Intake
    public IntakeArm() {}

    public void set(double speed) {
    
        intakeMover.set(speed);
    }
}
