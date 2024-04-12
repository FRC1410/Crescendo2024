package org.frc1410.crescendo2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.units.Units.RPM;

import static org.frc1410.crescendo2024.util.Constants.*;
import static org.frc1410.crescendo2024.util.IDs.*;

public class Storage implements Subsystem {
	private final CANSparkMax leftMotor = new CANSparkMax(STORAGE_LEFT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax rightMotor = new CANSparkMax(STORAGE_RIGHT_MOTOR_ID, MotorType.kBrushless);

	// Both use RPM
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

	public void setVelocity(Measure<Velocity<Angle>> velocity) {
		var motorVelocity = velocity.times(STORAGE_GEAR_RATIO);

		var feedforwardOutputLeft = this.feedforwardControllerLeft.calculate(motorVelocity.in(RPM));
		this.leftMotor.setVoltage(feedforwardOutputLeft);

		var feedforwardOutputRight = this.feedforwardControllerRight.calculate(motorVelocity.in(RPM));
		this.rightMotor.setVoltage(feedforwardOutputRight);
	}
}