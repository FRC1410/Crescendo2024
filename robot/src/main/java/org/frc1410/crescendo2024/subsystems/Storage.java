package org.frc1410.crescendo2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static org.frc1410.crescendo2024.util.IDs.*;
import static org.frc1410.crescendo2024.util.Constants.*;

public class Storage implements Subsystem {
	private final CANSparkMax leftMotor = new CANSparkMax(STORAGE_LEFT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax rightMotor = new CANSparkMax(STORAGE_RIGHT_MOTOR_ID, MotorType.kBrushless);

	private final SimpleMotorFeedforward feedforwardControllerLeft = new SimpleMotorFeedforward(STORAGE_LEFT_S, STORAGE_LEFT_V);
	private final SimpleMotorFeedforward feedforwardControllerRight = new SimpleMotorFeedforward(STORAGE_RIGHT_S, STORAGE_RIGHT_V);

	public Storage() {
		this.leftMotor.restoreFactoryDefaults();
		this.rightMotor.restoreFactoryDefaults();

		this.leftMotor.setIdleMode(IdleMode.kBrake);
		this.rightMotor.setIdleMode(IdleMode.kBrake);

		this.leftMotor.setInverted(STORAGE_LEFT_MOTOR_INVERTED);
		this.rightMotor.setInverted(STORAGE_RIGHT_MOTOR_INVERTED);
	}

	public void setSpeed(double speed) {
		this.leftMotor.set(speed);
		this.rightMotor.set(speed);
	}

	public void setRPM(double rpm) {
		var adjustedRPM = rpm * 12;

		var feedforwardOutputLeft = this.feedforwardControllerLeft.calculate(adjustedRPM);
		this.leftMotor.setVoltage(feedforwardOutputLeft);

		var feedforwardOutputRight = this.feedforwardControllerRight.calculate(adjustedRPM);
		this.rightMotor.setVoltage(feedforwardOutputRight );
	}
}