package org.frc1410.chargedup2023.Subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.frc1410.chargedup2023.util.Tuning.*;

import static org.frc1410.chargedup2023.util.Tuning.*;

public class SwerveModule extends SubsystemBase {

	private static final double maxAngularVel = 1;
	private static final double maxAngularAcc = 2 * Math.PI;  // raidians per sec squared
	private final CANSparkMax driveMotor;
	private final CANSparkMax turningMotor;

	private final CANCoder turningEncoder;

	private final PIDController drivePIDController = new PIDController(SWERVE_DRIVE_KP, SWERVE_DRIVE_KI, SWERVE_DRIVE_KD);

	private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(SWERVE_DRIVE_KS, SWERVE_DRIVE_KV, SWERVE_DRIVE_KA);
	private final SimpleMotorFeedforward turningFeedForward = new SimpleMotorFeedforward(TURN_KS, TURN_KV, TURN_KA);

	private final ProfiledPIDController turningPIDController = new ProfiledPIDController(
			SWERVE_TURNING_KP,
			SWERVE_TURNING_KI,
			SWERVE_TURNING_KD,
			new TrapezoidProfile.Constraints(maxAngularVel, maxAngularAcc)
	);

	public SwerveModule(int driveMotorID, int turningMotorID, int steeringEncoderID) {
		driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
		turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

		turningEncoder = new CANCoder(steeringEncoderID);

		turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(
			driveMotor.getEncoder().getVelocity(), new Rotation2d(turningEncoder.getAbsolutePosition())
		);
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			driveMotor.getEncoder().getPosition(), new Rotation2d(turningEncoder.getAbsolutePosition())
		);
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		SwerveModuleState state = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(turningEncoder.getAbsolutePosition()));

		final double driveOutput = drivePIDController.calculate(driveMotor.getEncoder().getVelocity(), state.speedMetersPerSecond);
		final double turnOutput = turningPIDController.calculate(turningEncoder.getAbsolutePosition(), state.angle.getRadians());

		double drivefeed = driveFeedForward.calculate(state.speedMetersPerSecond);
		double turnFeed = turningFeedForward.calculate(turningEncoder.getAbsolutePosition(), state.angle.getRadians());


		driveMotor.setVoltage(driveOutput + drivefeed);
		turningMotor.setVoltage(turnOutput + turnFeed);
	}
}