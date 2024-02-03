package org.frc1410.crescendo2024.subsystems;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

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

	// RPM
	private final SparkPIDController drivePIDController;

	// radians
	private final PIDController steeringPIDController = new PIDController(
		SWERVE_STEERING_KP,
		SWERVE_STEERING_KI,
		SWERVE_STEERING_KD
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
		this.driveMotor = new CANSparkMax(driveMotorID, CANSparkLowLevel.MotorType.kBrushless);
		this.driveMotor.restoreFactoryDefaults();
		this.driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		this.driveMotor.setInverted(driveMotorInverted);
		this.driveMotor.setSmartCurrentLimit(40);

		this.drivePIDController = driveMotor.getPIDController();
		this.drivePIDController.setP(SWERVE_DRIVE_KP);
		this.drivePIDController.setI(SWERVE_DRIVE_KI);
		this.drivePIDController.setD(SWERVE_DRIVE_KD);
		this.drivePIDController.setFF(SWERVE_DRIVE_KFF);

		this.steerMotor = new CANSparkMax(steeringMotorID, CANSparkLowLevel.MotorType.kBrushless);
		this.steerMotor.restoreFactoryDefaults();
		this.steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		this.steerMotor.setInverted(steerMotorInverted);
		this.steerMotor.setSmartCurrentLimit(30);

		this.driveEncoder = driveMotor.getEncoder();

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
		// System.out.println(optimized);
		var a = SwerveModule.metersPerSecondToEncoderRPM(optimized.speedMetersPerSecond);
		// System.out.println("aaa" + a);
		this.drivePIDController.setReference(a, CANSparkMax.ControlType.kVelocity);
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(this.getDriveVelocityMetersPerSecond(), this.getSteerPosition());
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			this.getDrivePositionMeters(), this.getSteerPosition()
		);
	}

	@Override
	public void periodic() {
		double steerPIDOutput = this.steeringPIDController.calculate(this.getSteerPosition().getRadians(), MathUtil.angleModulus(this.desiredState.angle.getRadians()));

		this.steerMotor.setVoltage(steerPIDOutput);

		this.desiredVel.set(this.desiredState.speedMetersPerSecond);
		this.actualVel.set(this.getDriveVelocityMetersPerSecond());
		
		this.desiredAngle.set(this.desiredState.angle.getRadians());
		this.actualAngle.set(this.getSteerPosition().getRadians());
	}

	private Rotation2d getSteerPosition() {
		var b = this.steerEncoder.getAbsolutePosition().getValue();
		var a = Rotation2d.fromRotations(b);
		// System.out.println(b);
		return a;
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