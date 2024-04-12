package org.frc1410.crescendo2024.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.RPM;

import static org.frc1410.crescendo2024.util.Constants.*;
import static org.frc1410.crescendo2024.util.IDs.*;
import static org.frc1410.crescendo2024.util.Tuning.*;

import org.frc1410.crescendo2024.util.NetworkTables;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

public class Shooter implements TickedSubsystem {
	private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Shooter");

    private final DoublePublisher rpmPub = NetworkTables.PublisherFactory(this.table, "Shooter RPM", 0);

	private final CANSparkMax leftMotor = new CANSparkMax(SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax rightMotor = new CANSparkMax(SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless);

	private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
	private final RelativeEncoder rightEncoder = rightMotor.getEncoder();

	// Both use RPM
	private final PIDController pidController = new PIDController(SHOOTER_LEFT_P, SHOOTER_LEFT_I, SHOOTER_LEFT_D);
	private final SimpleMotorFeedforward feedforwardController = new SimpleMotorFeedforward(SHOOTER_LEFT_S, SHOOTER_LEFT_V);

	public Measure<Velocity<Angle>> velocityAdjustment = STARTING_SHOOTER_VELOCITY_ADJUSTMENT;

	public Shooter() {
		this.leftMotor.restoreFactoryDefaults();
		this.rightMotor.restoreFactoryDefaults();

		this.leftMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
		this.rightMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

		this.leftMotor.setInverted(SHOOTER_LEFT_MOTOR_INVERTED);
		this.rightMotor.setInverted(SHOOTER_RIGHT_MOTOR_INVERTED);

		this.leftMotor.setSmartCurrentLimit(40);
		this.rightMotor.setSmartCurrentLimit(40);
	}

	public void setVelocity(Measure<Velocity<Angle>> velocity) {
		var feedforwardOutput = this.feedforwardController.calculate(velocity.in(RPM));
		var pidOutputLeft = this.pidController.calculate(this.leftEncoder.getVelocity(), velocity.in(RPM));
		var pidOutputRight = this.pidController.calculate(this.rightEncoder.getVelocity(), velocity.in(RPM));

		this.leftMotor.setVoltage(feedforwardOutput + pidOutputLeft);
		this.rightMotor.setVoltage(feedforwardOutput + pidOutputRight);
	}

	public Measure<Velocity<Angle>> getVelocity() {
		double leftRPM = this.leftEncoder.getVelocity();
		double rightRPM = this.rightEncoder.getVelocity();

		return RPM.of((leftRPM + rightRPM) / 2);
	}

	@Override
	public void periodic() {
		this.rpmPub.set(this.getVelocity().in(RPM));
	}
}