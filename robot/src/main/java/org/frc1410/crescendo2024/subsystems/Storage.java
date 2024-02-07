package org.frc1410.crescendo2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

	private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Shooter");
	private final DoublePublisher storageSpeedPub = NetworkTables.PublisherFactory(table, "Storage Speed", 1);
	private final DoubleSubscriber storageSpeed = NetworkTables.SubscriberFactory(table, table.getDoubleTopic("Storage Speed"));

	private final DoublePublisher storageSpeedRPM = NetworkTables.PublisherFactory(table, "Storage RPM", 1);

	public Storage() {
		storageLeftMotor.restoreFactoryDefaults();
		storageRightMotor.restoreFactoryDefaults();

		storageLeftMotor.setInverted(STORAGE_LEFT_MOTOR_INVERTED);
		storageRightMotor.setInverted(STORAGE_RIGHT_MOTOR_INVERTED);

		this.storageLeftEncoder = storageLeftMotor.getEncoder();
	}

	public void setSpeed(double speed) {
		storageLeftMotor.set(speed);
		storageRightMotor.set(speed);
	}

	public double getStorageSpeed() {
		return this.storageSpeed.get();
	}

	@Override
	public void periodic() {
		
		storageSpeedRPM.set(storageLeftEncoder.getVelocity());

		// manualShootTargetLeftRPM.set(shooterSpeed);
		// manualShootTargetRightRPM.set(shooterSpeed);
	}
}