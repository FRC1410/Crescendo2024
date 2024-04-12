package org.frc1410.crescendo2024.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.frc1410.crescendo2024.util.Constants.*;
import static org.frc1410.crescendo2024.util.Tuning.*;

import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

public class SwerveModule implements TickedSubsystem {
	private final TalonFX driveMotor;
	private final CANSparkMax steerMotor;

	private final CANcoder steerEncoder;

	private final PIDController steeringPIDController = new PIDController(
		SWERVE_STEERING_P,
		SWERVE_STEERING_I,
		SWERVE_STEERING_D
	);

	public SwerveModuleState desiredState = new SwerveModuleState();

	private final DoublePublisher desiredVel;
	private final DoublePublisher desiredAngle;

	private final DoublePublisher actualVel;
	private final DoublePublisher actualAngle;

	public SwerveModule(
		int driveMotorID, 
		int steeringMotorID, 
		int steeringEncoderID, 
		boolean driveMotorInverted, 
		boolean steerMotorInverted, 
		Measure<Angle> offset, 
		DoublePublisher desiredVel,
		DoublePublisher desiredAngle, 
		DoublePublisher actualVel, 
		DoublePublisher actualAngle
	) {
		this.driveMotor = new TalonFX(driveMotorID);

		var driveMotorConfig = new TalonFXConfiguration();
		
        driveMotorConfig.Slot0.kS = DRIVE_KS;
        driveMotorConfig.Slot0.kV = DRIVE_KV;

        driveMotorConfig.Slot0.kP = SWERVE_DRIVE_P;
        driveMotorConfig.Slot0.kI = SWERVE_DRIVE_I;
        driveMotorConfig.Slot0.kD = SWERVE_DRIVE_D;

		driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
		driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

		driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		driveMotorConfig.MotorOutput.Inverted = driveMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

		this.driveMotor.getConfigurator().apply(driveMotorConfig);

		this.steerMotor = new CANSparkMax(steeringMotorID, MotorType.kBrushless);
		this.steerMotor.restoreFactoryDefaults();
		this.steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		this.steerMotor.setInverted(steerMotorInverted);
		this.steerMotor.setSmartCurrentLimit(30);

		this.steerEncoder = new CANcoder(steeringEncoderID);
		var configurator = this.steerEncoder.getConfigurator();

		var steerEncoderConfig = new CANcoderConfiguration();
		steerEncoderConfig.MagnetSensor.MagnetOffset = offset.negate().in(Rotations);

		steerEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
		configurator.apply(steerEncoderConfig);

		this.steeringPIDController.enableContinuousInput(-Math.PI, Math.PI);

		this.desiredVel = desiredVel;
		this.desiredAngle = desiredAngle;

		this.actualVel = actualVel;
		this.actualAngle = actualAngle;
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		SwerveModuleState optimized = SwerveModuleState.optimize(
			desiredState,
			this.getSteerPosition()
		);

		this.desiredState = optimized;

		var request = new VelocityVoltage(
			SwerveModule.moduleVelocityToMotorAngularVelocity(
				MetersPerSecond.of(optimized.speedMetersPerSecond)
			)
			.in(RotationsPerSecond)
		);
		this.driveMotor.setControl(request);
	}

	public void drive(Measure<Voltage> voltage) {
		this.desiredState.angle = new Rotation2d();
		this.driveMotor.setVoltage(voltage.in(Volts));
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(
			this.getDriveVelocity(),
			this.getSteerPosition()
		);
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			this.getDrivePosition(),
			this.getSteerPosition()
		);
	}

	public Measure<Velocity<Angle>> getDriveAngularVelocity() {
		return RotationsPerSecond.of(this.driveMotor.getVelocity().getValue());
	}

	@Override
	public void periodic() {
		double steerPIDOutput = this.steeringPIDController.calculate(
			this.getSteerPosition().getRadians(),
			MathUtil.angleModulus(this.desiredState.angle.getRadians())
		);

		this.steerMotor.setVoltage(steerPIDOutput);

		this.desiredVel.set(this.desiredState.speedMetersPerSecond);
		this.actualVel.set(this.getDriveVelocity().in(MetersPerSecond));
		
		this.desiredAngle.set(this.desiredState.angle.getDegrees());
		this.actualAngle.set(this.getSteerPosition().getDegrees());
	}

	private Rotation2d getSteerPosition() {
		return Rotation2d.fromRotations(this.steerEncoder.getAbsolutePosition().getValue());
	}

	private Measure<Velocity<Distance>> getDriveVelocity() {
		return MetersPerSecond.of(driveMotor.getVelocity().getValue() * WHEEL_CIRCUMFERENCE.in(Meters)).divide(DRIVE_GEAR_RATIO);
	}

	private Measure<Distance> getDrivePosition() {
		return Meters.of(driveMotor.getPosition().getValue() * WHEEL_CIRCUMFERENCE.in(Meters)).divide(DRIVE_GEAR_RATIO);
	}

	private static Measure<Velocity<Angle>> moduleVelocityToMotorAngularVelocity(Measure<Velocity<Distance>> linearVelocity) {
		return RotationsPerSecond.of(linearVelocity.in(MetersPerSecond) / WHEEL_CIRCUMFERENCE.in(Meters));
	}
}