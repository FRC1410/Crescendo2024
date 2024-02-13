package org.frc1410.crescendo2024.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.frc1410.crescendo2024.util.NetworkTables;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

import static org.frc1410.crescendo2024.util.IDs.*;
import static org.frc1410.crescendo2024.util.Tuning.*;
import static org.frc1410.crescendo2024.util.Constants.*;


public class Shooter implements TickedSubsystem {

	private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Shooter");

	private final DoublePublisher leftActualVel = NetworkTables.PublisherFactory(table, "Left Actual Vel", 0);
	private final DoublePublisher rightActualVel = NetworkTables.PublisherFactory(table, "Right Actual Vel", 0);
	private final DoublePublisher shooterSpeedPub = NetworkTables.PublisherFactory(table, "Desired Shooter RPM", 3300);
	private final DoubleSubscriber desiredShooterRPM = NetworkTables.SubscriberFactory(table, table.getDoubleTopic("Desired Shooter RPM"));

	private final CANSparkMax shooterMotorRight = new CANSparkMax(SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax shooterMotorLeft = new CANSparkMax(SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);

	 private final SparkPIDController leftPIDController = shooterMotorLeft.getPIDController();
	 private final SparkPIDController rightPIDController = shooterMotorRight.getPIDController();

	private final RelativeEncoder shooterLeftEncoder = shooterMotorLeft.getEncoder();
	private final RelativeEncoder shooterRightEncoder = shooterMotorRight.getEncoder();

	private final PIDController pidController = new PIDController(SHOOTER_LEFT_P, SHOOTER_LEFT_I, SHOOTER_LEFT_D);
	private final SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(SHOOTER_LEFT_S, SHOOTER_LEFT_V);

	public double rpmAdjustment = 0;

	public Shooter() {
		shooterMotorLeft.restoreFactoryDefaults();
		shooterMotorRight.restoreFactoryDefaults();

		shooterMotorLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
		shooterMotorRight.setIdleMode(CANSparkBase.IdleMode.kBrake);

		shooterMotorLeft.setInverted(SHOOTER_LEFT_MOTOR_INVERTED);
		shooterMotorRight.setInverted(SHOOTER_RIGHT_MOTOR_INVERTED);

		shooterMotorLeft.setSmartCurrentLimit(40);
		shooterMotorLeft.setSmartCurrentLimit(40);
	}

	public void setRPM(double rpm) {

		var feedforwardOutput = this.feedforwardController.calculate(rpm);
		var pidOutputLeft = this.pidController.calculate(this.shooterLeftEncoder.getVelocity(), rpm);
		var pidOutputRight = this.pidController.calculate(this.shooterRightEncoder.getVelocity(), rpm);

		shooterMotorLeft.setVoltage(feedforwardOutput + pidOutputLeft);
		shooterMotorRight.setVoltage(feedforwardOutput + pidOutputRight);

		this.leftPIDController.setP(SHOOTER_LEFT_P);
		this.leftPIDController.setI(SHOOTER_LEFT_I);
		this.leftPIDController.setD(SHOOTER_LEFT_D);
		this.leftPIDController.setFF(SHOOTER_LEFT_FF);

		this.rightPIDController.setP(SHOOTER_RIGHT_P);
		this.rightPIDController.setI(SHOOTER_RIGHT_I);
		this.rightPIDController.setD(SHOOTER_RIGHT_D);
		this.rightPIDController.setFF(SHOOTER_RIGHT_FF);
	}

	public double getSpeed() {
		return this.desiredShooterRPM.get();
	}




	public void setVoltsLeft(double volts) {
		shooterMotorLeft.setVoltage(volts);
	}

	public void setVoltsRight(double volts) {
		shooterMotorRight.setVoltage(volts);
	}

	public double getRPMLeft() {
		return shooterLeftEncoder.getVelocity();
	}

	public double getRPMRight() {
		return shooterRightEncoder.getVelocity();
	}

	@Override
	public void periodic() {

		leftActualVel.set(shooterLeftEncoder.getVelocity());
		rightActualVel.set(shooterRightEncoder.getVelocity());
	}
}