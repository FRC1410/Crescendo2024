package org.frc1410.crescendo2024.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static org.frc1410.crescendo2024.util.IDs.*;

public class Climber implements Subsystem {
	private final CANSparkMax leftClimber = new CANSparkMax(CLIMBER_LEFT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax rightClimber = new CANSparkMax(CLIMBER_RIGHT_MOTOR_ID, MotorType.kBrushless);

	public Climber() {
		this.leftClimber.restoreFactoryDefaults();
		this.rightClimber.restoreFactoryDefaults();

		this.leftClimber.setIdleMode(CANSparkBase.IdleMode.kBrake);
		this.rightClimber.setIdleMode(CANSparkBase.IdleMode.kBrake);

		this.leftClimber.setSmartCurrentLimit(40);
		this.rightClimber.setSmartCurrentLimit(40);
	}

	public void setLeftClimberSpeed(double speed) {
		this.leftClimber.set(speed);
	}

	public void setRightClimberSpeed(double speed) {
		this.rightClimber.set(speed);
	}
}