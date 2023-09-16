package org.frc1410.chargedup2023.Subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;

import static org.frc1410.chargedup2023.util.Constants.*;
import static org.frc1410.chargedup2023.util.Tuning.*;

import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

public class SwerveModule implements TickedSubsystem {

	private final CANSparkMax driveMotor;
	private final CANSparkMax steerMotor;

	private final RelativeEncoder driveEncoder;
	private final CANCoder steerEncoder;

	private final PIDController drivePIDController = new PIDController(SWERVE_DRIVE_KP, SWERVE_DRIVE_KI, SWERVE_DRIVE_KD);
	private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(SWERVE_DRIVE_KS, SWERVE_DRIVE_KV);
	
	// private final PIDController turningPIDController = new PIDController(SWERVE_STEERING_KP, SWERVE_STEERING_KI, SWERVE_STEERING_KD);
	private final ProfiledPIDController turningPIDController = new ProfiledPIDController(
		SWERVE_STEERING_KP, 
		SWERVE_STEERING_KI, 
		SWERVE_STEERING_KD, 
		new TrapezoidProfile.Constraints(Math.PI, 2 * Math.PI)
	);
	private final SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(1, 0.5);

	public SwerveModuleState desiredState = new SwerveModuleState();

	private final DoublePublisher voltage;

	public SwerveModule(int driveMotorID, int steeringMotorID, int steeringEncoderID, boolean driveMotorInverted, boolean steerMotorInverted, double offset, DoublePublisher voltage) {
		driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
		driveMotor.restoreFactoryDefaults();
		driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		driveMotor.setInverted(driveMotorInverted);
		driveMotor.setSmartCurrentLimit(40);

		steerMotor = new CANSparkMax(steeringMotorID, MotorType.kBrushless);
		steerMotor.restoreFactoryDefaults();
		steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		steerMotor.setInverted(steerMotorInverted);
		steerMotor.setSmartCurrentLimit(30);

		driveEncoder = driveMotor.getEncoder();

		steerEncoder = new CANCoder(steeringEncoderID);
		steerEncoder.configMagnetOffset(-offset);

		turningPIDController.enableContinuousInput(-Math.PI, Math.PI);	
		
		this.voltage = voltage;

		// ?
		this.turningPIDController.reset(getSteerPosition());
	}

	@Override
	public void periodic() {
		double driveFeedOutput = driveFeedforward.calculate(desiredState.speedMetersPerSecond);
		double drivePIDOutput = drivePIDController.calculate(getDriveVelocityMetersPerSecond(), desiredState.speedMetersPerSecond);

		double steerPIDOutput = turningPIDController.calculate(getSteerPosition(), MathUtil.angleModulus(desiredState.angle.getRadians()));
		double turnFeedOutput = turningFeedforward.calculate(turningPIDController.getSetpoint().velocity);


		driveMotor.setVoltage(driveFeedOutput + drivePIDOutput);
		steerMotor.setVoltage(steerPIDOutput + turnFeedOutput);
		voltage.set(Units.radiansToDegrees(getSteerPosition()));
		// this.steerMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		SwerveModuleState optimized = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(steerEncoder.getAbsolutePosition()));
		this.desiredState = optimized;
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			getDrivePositionMeters(), new Rotation2d(getSteerPosition())
		);
	}

	public double getDriveVelocityMetersPerSecond() {
		return (driveEncoder.getVelocity() * DRIVE_ROTATIONS_TO_METERES) / 60;
	}

	public double getDrivePositionMeters() {
		return driveEncoder.getPosition() * DRIVE_ROTATIONS_TO_METERES;
	}

	public double getSteerPosition() {
		double rem = steerEncoder.getAbsolutePosition() % 360;

		double res;

		if (rem > 180) {
			res = rem - 360;
		} else if (rem <= -180) {
			res = rem + 360;
		} else {
			res = rem;
		}

		return Units.degreesToRadians(res);
	}
}