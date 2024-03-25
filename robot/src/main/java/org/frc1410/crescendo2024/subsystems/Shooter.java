package org.frc1410.crescendo2024.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static org.frc1410.crescendo2024.util.IDs.*;
import static org.frc1410.crescendo2024.util.Tuning.*;
import static org.frc1410.crescendo2024.util.Constants.*;

public class Shooter implements Subsystem {
	private final CANSparkMax leftMotor = new CANSparkMax(SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax rightMotor = new CANSparkMax(SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless);

	private final SparkPIDController leftPIDController = leftMotor.getPIDController();
	private final SparkPIDController rightPIDController = rightMotor.getPIDController();

	private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
	private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

	private final PIDController pidController = new PIDController(SHOOTER_LEFT_P, SHOOTER_LEFT_I, SHOOTER_LEFT_D);
	private final SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(SHOOTER_LEFT_S, SHOOTER_LEFT_V);

	public double rpmAdjustment = STARTING_SHOOTER_RPM_ADJUSTMENT;

	public Shooter() {
		this.leftMotor.restoreFactoryDefaults();
		this.rightMotor.restoreFactoryDefaults();

		this.leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
		this.rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

		this.leftMotor.setInverted(SHOOTER_LEFT_MOTOR_INVERTED);
		this.rightMotor.setInverted(SHOOTER_RIGHT_MOTOR_INVERTED);

		this.leftMotor.setSmartCurrentLimit(40);
		this.rightMotor.setSmartCurrentLimit(40);

		this.leftPIDController.setP(SHOOTER_LEFT_P);
		this.leftPIDController.setI(SHOOTER_LEFT_I);
		this.leftPIDController.setD(SHOOTER_LEFT_D);
		this.leftPIDController.setFF(SHOOTER_LEFT_FF);

		this.rightPIDController.setP(SHOOTER_RIGHT_P);
		this.rightPIDController.setI(SHOOTER_RIGHT_I);
		this.rightPIDController.setD(SHOOTER_RIGHT_D);
		this.rightPIDController.setFF(SHOOTER_RIGHT_FF);
	}

	public void setRPM(double rpm) {
		var feedforwardOutput = this.feedforwardController.calculate(rpm);
		var pidOutputLeft = this.pidController.calculate(this.leftEncoder.getVelocity(), rpm);
		var pidOutputRight = this.pidController.calculate(this.leftEncoder.getVelocity(), rpm);

		this.leftMotor.setVoltage(feedforwardOutput + pidOutputLeft);
		this.rightMotor.setVoltage(feedforwardOutput + pidOutputRight);
	}

	public double getRPM() {
		double leftRPM = this.leftEncoder.getVelocity();
		double rightRPM = this.rightEncoder.getVelocity();

		return (leftRPM + rightRPM) / 2;
	}
}