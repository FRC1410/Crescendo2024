package org.frc1410.crescendo2024.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
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
	private final CANSparkMax driveMotor;
	private final CANSparkMax steerMotor;

	private final RelativeEncoder driveEncoder;
	private final CANcoder steerEncoder;

	private final SparkPIDController drivePIDController;

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
		this.driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
		this.driveMotor.restoreFactoryDefaults();
		this.driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		this.driveMotor.setInverted(driveMotorInverted);
		this.driveMotor.setSmartCurrentLimit(40);

		this.steerMotor = new CANSparkMax(steeringMotorID, MotorType.kBrushless);
		this.steerMotor.restoreFactoryDefaults();
		this.steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		this.steerMotor.setInverted(steerMotorInverted);
		this.steerMotor.setSmartCurrentLimit(30);

		this.driveEncoder = this.driveMotor.getEncoder();

		drivePIDController = this.driveMotor.getPIDController();

		drivePIDController.setP(SWERVE_DRIVE_P);
		drivePIDController.setI(SWERVE_DRIVE_I);
		drivePIDController.setD(SWERVE_DRIVE_D);
		drivePIDController.setFF(SWERVE_DRIVE_FF);

		this.steerEncoder = new CANcoder(steeringEncoderID);
		var configurator = this.steerEncoder.getConfigurator();

		var config = new CANcoderConfiguration();
		config.MagnetSensor.MagnetOffset = -Rotation2d.fromRadians(offset).getRotations();

		config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
		configurator.apply(config);

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

		this.drivePIDController.setReference(SwerveModule.metersPerSecondToEncoderRPM(optimized.speedMetersPerSecond), CANSparkBase.ControlType.kVelocity);
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
		return ((driveEncoder.getVelocity() / 60) * WHEEL_CIRCUMFERENCE) / DRIVE_GEAR_RATIO;
	}

	private double getDrivePositionMeters() {
		return (driveEncoder.getPosition() * WHEEL_CIRCUMFERENCE) / DRIVE_GEAR_RATIO;
	}

	private static double metersPerSecondToEncoderRPM(double metersPerSecond) {
		return ((metersPerSecond * 60) / WHEEL_CIRCUMFERENCE) * DRIVE_GEAR_RATIO;
	}
}