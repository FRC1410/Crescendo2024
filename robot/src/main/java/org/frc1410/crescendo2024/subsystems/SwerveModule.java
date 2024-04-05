package org.frc1410.crescendo2024.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;

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
		double offset, 
		DoublePublisher desiredVel,
		DoublePublisher desiredAngle, 
		DoublePublisher actualVel, 
		DoublePublisher actualAngle
	) {
		this.driveMotor = new TalonFX(driveMotorID);

		var driveMotorConfig = new TalonFXConfiguration();

		// TODO
        driveMotorConfig.Slot0.kS = 0;
        driveMotorConfig.Slot0.kV = 0.0192;

        driveMotorConfig.Slot0.kP = SWERVE_DRIVE_P;
        driveMotorConfig.Slot0.kI = SWERVE_DRIVE_I;
        driveMotorConfig.Slot0.kD = SWERVE_DRIVE_D;

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
		steerEncoderConfig.MagnetSensor.MagnetOffset = -Rotation2d.fromRadians(offset).getRotations();

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

		// this.drivePIDController.setReference(SwerveModule.metersPerSecondToEncoderRPM(optimized.speedMetersPerSecond), CANSparkBase.ControlType.kVelocity);

		var request = new VelocityVoltage(SwerveModule.metersPerSecondToEncoderRPS(optimized.speedMetersPerSecond));

		this.driveMotor.setControl(request);


		// this.drivePIDController.
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(
			this.getDriveVelocityMetersPerSecond(),
			this.getSteerPosition()
		);
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			this.getDrivePositionMeters(),
			this.getSteerPosition()
		);
	}

	@Override
	public void periodic() {
		double steerPIDOutput = this.steeringPIDController.calculate(
			this.getSteerPosition().getRadians(),
			MathUtil.angleModulus(this.desiredState.angle.getRadians())
		);

		this.steerMotor.setVoltage(steerPIDOutput);

		this.desiredVel.set(this.desiredState.speedMetersPerSecond);
		this.actualVel.set(this.getDriveVelocityMetersPerSecond());
		
		this.desiredAngle.set(this.desiredState.angle.getRadians());
		this.actualAngle.set(this.getSteerPosition().getRadians());
	}

	private Rotation2d getSteerPosition() {
		return Rotation2d.fromRotations(this.steerEncoder.getAbsolutePosition().getValue());
	}

	private double getDriveVelocityMetersPerSecond() {
		return ((driveMotor.getVelocity().getValue()) * WHEEL_CIRCUMFERENCE) / DRIVE_GEAR_RATIO;
	}

	private double getDrivePositionMeters() {
		return (driveMotor.getPosition().getValue() * WHEEL_CIRCUMFERENCE) / DRIVE_GEAR_RATIO;
	}

	private static double metersPerSecondToEncoderRPS(double metersPerSecond) {
		return ((metersPerSecond) / WHEEL_CIRCUMFERENCE) * DRIVE_GEAR_RATIO;
	}
}