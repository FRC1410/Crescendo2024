package org.frc1410.chargedup2023.Subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.frc1410.chargedup2023.util.Constants.*;
import static org.frc1410.chargedup2023.util.Tuning.*;

public class SwerveModule extends SubsystemBase {

	private final CANSparkMax driveMotor;
	private final CANSparkMax steerMotor;

	private final RelativeEncoder driveEncoder;
	private final CANCoder steerEncoder;

	private final PIDController drivePIDController = new PIDController(SWERVE_DRIVE_KP, SWERVE_DRIVE_KI, SWERVE_DRIVE_KD);

	private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(SWERVE_DRIVE_KS, SWERVE_DRIVE_KV, SWERVE_DRIVE_KA);
	private final SimpleMotorFeedforward turningFeedForward = new SimpleMotorFeedforward(TURN_KS, TURN_KV, TURN_KA);

	private final ProfiledPIDController turningPIDController = new ProfiledPIDController(
			SWERVE_TURNING_KP,
			SWERVE_TURNING_KI,
			SWERVE_TURNING_KD,
			new TrapezoidProfile.Constraints(MAX_ANGULAR_VEL, MAX_ANGULAR_ACC)
	);

	public SwerveModule(int driveMotorID, int steeringMotorID, int steeringEncoderID) {
		driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
		steerMotor = new CANSparkMax(steeringMotorID, MotorType.kBrushless);

		driveEncoder = driveMotor.getEncoder();
		steerEncoder = new CANCoder(steeringEncoderID);

		turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

		driveMotor.restoreFactoryDefaults();
		driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

		steerMotor.restoreFactoryDefaults();
		steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
	}

	public double getDriveVel() {
		return driveEncoder.getVelocity() / DRIVING_GEAR_RATIO;
	}

	public double getDrivePosition() {
		return driveEncoder.getPosition() / DRIVING_GEAR_RATIO;
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

	public void setDesiredState(SwerveModuleState desiredState) {
		SwerveModuleState state = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(steerEncoder.getAbsolutePosition()));

		final double driveOutput = drivePIDController.calculate(getDriveVel(), state.speedMetersPerSecond);
		final double turnOutput = turningPIDController.calculate(steerEncoder.getAbsolutePosition(), state.angle.getRadians());

		double driveFeed = driveFeedForward.calculate(state.speedMetersPerSecond);
		double turnFeed = turningFeedForward.calculate(steerEncoder.getAbsolutePosition(), state.angle.getRadians());

		driveMotor.setVoltage(driveOutput + driveFeed);
		steerMotor.setVoltage(turnOutput + turnFeed);
	}
}