package org.frc1410.chargedup2023.Subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static org.frc1410.chargedup2023.util.Constants.*;
import static org.frc1410.chargedup2023.util.Tuning.*;

public class SwerveModule implements Subsystem {

	private final CANSparkMax driveMotor;
	private final CANSparkMax steerMotor;

	private final RelativeEncoder driveEncoder;
	private final CANCoder steerEncoder;

	private final PIDController drivePIDController = new PIDController(SWERVE_DRIVE_KP, SWERVE_DRIVE_KI, SWERVE_DRIVE_KD);

	private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(SWERVE_DRIVE_KS, SWERVE_DRIVE_KV);
	private final SimpleMotorFeedforward turningFeedForward = new SimpleMotorFeedforward(STEER_KS, STEER_KV);
	public SwerveModuleState desiredState = new SwerveModuleState();

	private final ProfiledPIDController turningPIDController = new ProfiledPIDController(
			SWERVE_STEERING_KP,
			SWERVE_STEERING_KI,
			SWERVE_STEERING_KD,
			new TrapezoidProfile.Constraints(MAX_ANGULAR_VEL, MAX_ANGULAR_ACC)
	);

	private double offset = 0;

	public SwerveModule(int driveMotorID, int steeringMotorID, int steeringEncoderID, boolean motorInverted) {

		driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
		steerMotor = new CANSparkMax(steeringMotorID, MotorType.kBrushless);

		driveEncoder = driveMotor.getEncoder();
		steerEncoder = new CANCoder(steeringEncoderID);

		steerEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
		// steerEncoder.configMagnetOffset(magnetOffset);

		turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

		driveMotor.restoreFactoryDefaults();
		driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		steerMotor.restoreFactoryDefaults();
		steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		System.out.println("steer encoder position PRE:" + getEncoderValue());
		steerEncoder.setPosition(0);
		System.out.println("steer encoder position POST:" + getEncoderValue());

		steerMotor.setInverted(motorInverted);
	}

	public double getDriveVel() {
//		return driveEncoder.getVelocity() / DRIVING_GEAR_RATIO / 60 * 0.102 * 2 * Math.PI;
		return driveEncoder.getVelocity() * DRIVE_ENCODER_CONSTANT / 60;
	}

	public double getDrivePosition() {
//		return driveEncoder.getPosition() / DRIVING_GEAR_RATIO / 60 * 0.102 * 2 * Math.PI;
		return driveEncoder.getPosition() * DRIVE_ENCODER_CONSTANT;
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(
			getDriveVel(), new Rotation2d(steerEncoder.getAbsolutePosition())
		);
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			getDrivePosition(), new Rotation2d(steerEncoder.getAbsolutePosition())
		);
	}

	public double getEncoderValue() {
		return steerEncoder.getPosition();
	}

	// public void setEncoderValue(double position) {
	// 	steerEncoder.setPosition(position);
	// }

	public void setEncoderOffset(double offset) {
		steerEncoder.setPosition(steerEncoder.getAbsolutePosition() - offset);
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		// SwerveModuleState optimized = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(steerEncoder.getAbsolutePosition()));
		this.desiredState = desiredState;
	}

	public void setDriveCoast() {
		driveMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
	}

	public void setDriveBreak() {
		driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

//	@Override
	public void quoteUnquotePeriodic() {
		// if (true) return;
//			System.out.println("state.angle = " + state.angle);
//			System.out.println("state.speed = " + state.speedMetersPerSecond);

		final double driveOutput = drivePIDController.calculate(getDriveVel(), desiredState.speedMetersPerSecond);
		final double turnOutput = turningPIDController.calculate(Units.degreesToRadians(steerEncoder.getPosition()), desiredState.angle.getRadians());

		double driveFeed = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
		// double turnFeed = turningFeedForward.calculate(turningPIDController.getSetpoint().velocity);

		// driveMotor.setVoltage(driveFeed + driveOutput);
		steerMotor.setVoltage(turnOutput);
	}

	public void runSteeringMotor() {
		// new NullPointerException("Running steer motor with inverted set to " + steerMotor.getInverted()).printStackTrace();
		// steerMotor.set(0.5);
	}
}