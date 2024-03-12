package org.frc1410.crescendo2024.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static org.frc1410.crescendo2024.util.IDs.*;
import static org.frc1410.crescendo2024.util.Constants.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

public class IntakeBar implements Subsystem{
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("IntakeBar");
    private final CANSparkMax intakeBar = new CANSparkMax(INTAKE_BAR_MOTOR_ID, MotorType.kBrushless);

    public IntakeBar(){
        intakeBar.restoreFactoryDefaults();
        intakeBar.setIdleMode(CANSparkBase.IdleMode.kBrake);
        intakeBar.setInverted(INTAKE_BAR_MOTOR_INVERTED);
    }

    public void setSpeed(double speed){
        intakeBar.set(speed);
    }



}


