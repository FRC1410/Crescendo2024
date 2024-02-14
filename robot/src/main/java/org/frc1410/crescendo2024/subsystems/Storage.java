package org.frc1410.crescendo2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static org.frc1410.crescendo2024.util.IDs.*;

import org.frc1410.crescendo2024.util.NetworkTables;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

import static org.frc1410.crescendo2024.util.Constants.*;


public class Storage implements TickedSubsystem {

	private final CANSparkMax storageLeftMotor = new CANSparkMax(STORAGE_LEFT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax storageRightMotor = new CANSparkMax(STORAGE_RIGHT_MOTOR_ID, MotorType.kBrushless);

	private final RelativeEncoder storageLeftEncoder;
	private final RelativeEncoder storageRightEncoder;

	private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Shooter");
	private final DoublePublisher storageSpeedPub = NetworkTables.PublisherFactory(table, "Desired Storage RPM", 0);
	private final DoubleSubscriber desiredStorageRPM = NetworkTables.SubscriberFactory(table, table.getDoubleTopic("Desired Storage RPM"));

	private final DoublePublisher storageSpeedRPMLeft = NetworkTables.PublisherFactory(table, "Storage RPM Left", 1);
	private final DoublePublisher storageSpeedRPMRight = NetworkTables.PublisherFactory(table, "Storage RPM Right", 1);

	private final PIDController pidControllerLeft = new PIDController(0, 0, 0);
	private final PIDController pidControllerRight = new PIDController(0, 0, 0);
	private final SimpleMotorFeedforward feedforwardControllerLeft = new SimpleMotorFeedforward(STORAGE_LEFT_S, STORAGE_LEFT_V);
	private final SimpleMotorFeedforward feedforwardControllerRight = new SimpleMotorFeedforward(STORAGE_RIGHT_S, STORAGE_RIGHT_V);

	public Storage() {
		storageLeftMotor.restoreFactoryDefaults();
		storageRightMotor.restoreFactoryDefaults();

		storageLeftMotor.setIdleMode(IdleMode.kBrake);
		storageRightMotor.setIdleMode(IdleMode.kBrake);

		storageLeftMotor.setInverted(STORAGE_LEFT_MOTOR_INVERTED);
		storageRightMotor.setInverted(STORAGE_RIGHT_MOTOR_INVERTED);

		this.storageLeftEncoder = storageLeftMotor.getEncoder();
		this.storageRightEncoder = storageRightMotor.getEncoder();
	}

	public void setSpeed(double speed) {
		storageLeftMotor.set(speed);
		storageRightMotor.set(speed);
	}

	public void setRPM(double rpm) {
		var adjustedRPM = rpm * 12;

		var feedforwardOutputLeft = this.feedforwardControllerLeft.calculate(adjustedRPM);
		var pidOutputLeft = this.pidControllerLeft.calculate(this.storageLeftEncoder.getVelocity(), adjustedRPM);
		this.storageLeftMotor.setVoltage(feedforwardOutputLeft + pidOutputLeft);

		var feedforwardOutputRight = this.feedforwardControllerRight.calculate(adjustedRPM);
		var pidOutputRight = this.pidControllerRight.calculate(this.storageRightEncoder.getVelocity(), adjustedRPM);
		this.storageRightMotor.setVoltage(feedforwardOutputRight + pidOutputRight);
	}

	public double getStorageSpeed() {
		return this.desiredStorageRPM.get();
	}

	@Override
	public void periodic() {
		
		storageSpeedRPMLeft.set(storageLeftEncoder.getVelocity() / 12);
		storageSpeedRPMRight.set(storageRightEncoder.getVelocity() / 12);

		// manualShootTargetLeftRPM.set(shooterSpeed);
		// manualShootTargetRightRPM.set(shooterSpeed);
	}
}