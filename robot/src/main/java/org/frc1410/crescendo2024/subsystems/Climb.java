package org.frc1410.crescendo2024.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static org.frc1410.crescendo2024.util.IDs.*;

public class Climb implements Subsystem {

	private final CANSparkMax leftClimber = new CANSparkMax(CLIMB_LEFT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax rightClimber = new CANSparkMax(CLIMB_RIGHT_MOTOR_ID, MotorType.kBrushless);

	public Climb() {

		leftClimber.restoreFactoryDefaults();
		rightClimber.restoreFactoryDefaults();

		leftClimber.setIdleMode(CANSparkBase.IdleMode.kBrake);
		rightClimber.setIdleMode(CANSparkBase.IdleMode.kBrake);

		leftClimber.setSmartCurrentLimit(40);
		rightClimber.setSmartCurrentLimit(40);
	}

	public void setLeftClimberSpeed(double leftClimberSpeed) {
		leftClimber.set(leftClimberSpeed);
	}

	public void setRightClimberSpeed(double rightClimberSpeed) {
		rightClimber.set(rightClimberSpeed);
	}
}