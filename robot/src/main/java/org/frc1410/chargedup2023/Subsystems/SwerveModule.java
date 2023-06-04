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
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

import static org.frc1410.chargedup2023.util.Constants.*;
import static org.frc1410.chargedup2023.util.Tuning.*;

public class SwerveModule implements Subsystem {

	private final CANSparkMax driveMotor;
	private final CANSparkMax steerMotor;

	private final RelativeEncoder driveEncoder;
	private final CANCoder steerEncoder;

	private final PIDController drivePIDController = new PIDController(SWERVE_DRIVE_KP, SWERVE_DRIVE_KI, SWERVE_DRIVE_KD);

	private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(SWERVE_DRIVE_KS, SWERVE_DRIVE_KV, SWERVE_DRIVE_KA);
	private final SimpleMotorFeedforward turningFeedForward = new SimpleMotorFeedforward(STEER_KS, STEER_KV, STEER_KA);
	private SwerveModuleState desiredState;

	private final ProfiledPIDController turningPIDController = new ProfiledPIDController(
			SWERVE_STEERING_KP,
			SWERVE_STEERING_KI,
			SWERVE_STEERING_KD,
			new TrapezoidProfile.Constraints(MAX_ANGULAR_VEL, MAX_ANGULAR_ACC)
	);

	private double offset = 0;

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

		System.out.println("steer encoder position PRE:" + getEncoderValue());
		steerEncoder.setPosition(0);
		System.out.println("steer encoder position POST:" + getEncoderValue());
	}

	public double getDriveVel() {
//		return driveEncoder.getVelocity() / DRIVING_GEAR_RATIO / 60 * 0.102 * 2 * Math.PI;
		return driveEncoder.getVelocity() / DRIVING_GEAR_RATIO / 60;
	}

	public double getDrivePosition() {
//		return driveEncoder.getPosition() / DRIVING_GEAR_RATIO / 60 * 0.102 * 2 * Math.PI;
		return driveEncoder.getPosition() / DRIVING_GEAR_RATIO;
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(
			getDriveVel(), new Rotation2d(steerEncoder.getPosition())
		);
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			getDrivePosition(), new Rotation2d(steerEncoder.getPosition())
		);
	}

	public double getEncoderValue() {
		return steerEncoder.getPosition();
	}

	public void setEncoderValue() {
		steerEncoder.setPosition(0);
	}

	public void setDesiredState(SwerveModuleState desiredState) {
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
		if(desiredState != null) {
			SwerveModuleState state = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(steerEncoder.getPosition()));
//			System.out.println("state.angle = " + state.angle);
//			System.out.println("state.speed = " + state.speedMetersPerSecond);


			final double driveOutput = drivePIDController.calculate(getDriveVel(), state.speedMetersPerSecond);
			final double turnOutput = turningPIDController.calculate(steerEncoder.getPosition(), state.angle.getRadians());

			double driveFeed = driveFeedForward.calculate(state.speedMetersPerSecond);
			double turnFeed = turningFeedForward.calculate(steerEncoder.getPosition(), state.angle.getRadians());

			driveMotor.setVoltage((driveOutput + driveFeed) * 12);
//			System.out.println("drive = " + driveOutput + driveFeed);
			steerMotor.setVoltage((turnOutput + turnFeed) * 12);
//			System.out.println("steer = " + turnOutput + turnFeed);
		}
	}
}