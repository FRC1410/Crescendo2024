package org.frc1410.crescendo2024.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
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
	private final DoublePublisher shooterSpeedPub = NetworkTables.PublisherFactory(table, "Shooter Speed", 1);
	private final DoubleSubscriber shooterSpeed = NetworkTables.SubscriberFactory(table, table.getDoubleTopic("Shooter Speed"));

	private final CANSparkMax shooterMotorRight = new CANSparkMax(SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax shooterMotorLeft = new CANSparkMax(SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);

	private final SparkPIDController leftPIDController = shooterMotorLeft.getPIDController();
	private final SparkPIDController rightPIDController = shooterMotorRight.getPIDController();

	private final RelativeEncoder shooterLeftEncoder = shooterMotorLeft.getEncoder();
	private final RelativeEncoder shooterRightEncoder = shooterMotorRight.getEncoder();

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

		this.leftPIDController.setP(SHOOTER_LEFT_KP);
		this.leftPIDController.setI(SHOOTER_LEFT_KI);
		this.leftPIDController.setD(SHOOTER_LEFT_KD);
		this.leftPIDController.setFF(SHOOTER_LEFT_KFF);

		this.rightPIDController.setP(SHOOTER_RIGHT_KP);
		this.rightPIDController.setI(SHOOTER_RIGHT_KI);
		this.rightPIDController.setD(SHOOTER_RIGHT_KD);
		this.rightPIDController.setFF(SHOOTER_RIGHT_KFF);
	}

	public void setRPM(double velocity) {
		// leftPIDController.setReference(velocity, CANSparkBase.ControlType.kVelocity);
		// rightPIDController.setReference(velocity, CANSparkBase.ControlType.kVelocity);
		shooterMotorLeft.set(velocity);
		shooterMotorRight.set(velocity);
	}

	public double getSpeed() {
		return this.shooterSpeed.get();
	}



	// public double getTargetRPM() {
	// 	return shooterSpeed;
	// }

	@Override
	public void periodic() {
		
		leftActualVel.set(shooterLeftEncoder.getVelocity());
		rightActualVel.set(shooterRightEncoder.getVelocity());

		// manualShootTargetLeftRPM.set(shooterSpeed);
		// manualShootTargetRightRPM.set(shooterSpeed);
	}
}