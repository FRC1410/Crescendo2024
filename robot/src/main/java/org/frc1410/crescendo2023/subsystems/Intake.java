package org.frc1410.crescendo2023.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static org.frc1410.crescendo2023.util.IDs.*;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Intake implements Subsystem {
	private final CANSparkMax intakeMotorFront = new CANSparkMax(INTAKE_FRONT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax intakeMotorBack = new CANSparkMax(INTAKE_BACK_MOTOR_ID, MotorType.kBrushless);
	public Intake () {
		intakeMotorFront.restoreFactoryDefaults();
		intakeMotorBack.restoreFactoryDefaults();

		intakeMotorFront.setInverted(true);
		intakeMotorBack.setInverted(false);
		// One motor will be inverted (front) and the other will go in normal direction (opposite)

		intakeMotorFront.setIdleMode(CANSparkBase.IdleMode.kBrake);
		intakeMotorBack.setIdleMode(CANSparkBase.IdleMode.kBrake);
	}

	public void setSpeed(double speed) {
		intakeMotorFront.set(speed);
		intakeMotorBack.set(speed);

	}
}
