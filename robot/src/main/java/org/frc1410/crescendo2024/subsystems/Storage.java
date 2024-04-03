package org.frc1410.crescendo2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import static org.frc1410.crescendo2024.util.IDs.*;

import org.frc1410.crescendo2024.util.NetworkTables;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

import static org.frc1410.crescendo2024.util.Constants.*;

public class Storage implements TickedSubsystem {
	private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Storage");

    private final DoublePublisher rpmPub = NetworkTables.PublisherFactory(this.table, "Storage RPM", 0);

	private final CANSparkMax leftMotor = new CANSparkMax(STORAGE_LEFT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax rightMotor = new CANSparkMax(STORAGE_RIGHT_MOTOR_ID, MotorType.kBrushless);

	private final SimpleMotorFeedforward feedforwardControllerLeft = new SimpleMotorFeedforward(STORAGE_LEFT_S, STORAGE_LEFT_V);
	private final SimpleMotorFeedforward feedforwardControllerRight = new SimpleMotorFeedforward(STORAGE_RIGHT_S, STORAGE_RIGHT_V);

	private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
	private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

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

	public double getRPM() {
		double leftRPM = this.leftEncoder.getVelocity();
		double rightRPM = this.rightEncoder.getVelocity();

		return (leftRPM + rightRPM) / 2;
	}

	@Override
	public void periodic() {
		this.rpmPub.set(this.getRPM());
	}
}