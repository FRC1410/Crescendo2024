package org.frc1410.chargedup2023.Subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

import static org.frc1410.chargedup2023.util.Tuning.*;

public class SwerveModule implements TickedSubsystem {
	private static final double wheelRaidus = 1;
	private static final double encoderResoulution = 4092;
	private static final double maxAngularVel = 0;
	private static final double maxAngularAcc = 2 * Math.PI;  // raidians per sec squared
	private final CANSparkMax driveMotor;
	private final CANSparkMax turningMotor;

	private final CANCoder drivingEncoder;
	private final CANCoder turningEncoder;

	private final PIDController drivePIDController = new PIDController(SWERVE_DRIVE_KP, SWERVE_DRIVE_KI, SWERVE_DRIVE_KD);

	private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(DRIVE_KS, DRIVE_KV, DRIVE_KA);
	private final SimpleMotorFeedforward turningFeedForward = new SimpleMotorFeedforward(TURN_KS, TURN_KV, TURN_KA);

	private final ProfiledPIDController turningPIDController = new ProfiledPIDController(
			SWERVE_TURNING_KP,
			SWERVE_TURNING_KI,
			SWERVE_TURNING_KD,
			new TrapezoidProfile.Constraints(maxAngularVel, maxAngularAcc)
	);

	public SwerveModule(int driveID, int turningID, int drivingEncoderID, int steeringEncoderID) {
		driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
		turningMotor = new CANSparkMax(turningID, MotorType.kBrushless);

		drivingEncoder = new CANCoder(drivingEncoderID);
		turningEncoder = new CANCoder(steeringEncoderID);

		drivingEncoder.getPosition();
		turningEncoder.getAbsolutePosition();

		turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
	}

	public SwerveModuleState getPosition() {
		return new SwerveModuleState(drivingEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		SwerveModuleState state = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(turningEncoder.getAbsolutePosition()));

		final double driveOutput = drivePIDController.calculate(drivingEncoder.getPosition(), state.speedMetersPerSecond);
		final double turnOutput = turningPIDController.calculate(turningEncoder.getAbsolutePosition(), state.angle.getRadians());

		double drivefeed = driveFeedForward.calculate(state.speedMetersPerSecond);
		double turnFeed = turningFeedForward.calculate(turningEncoder.getAbsolutePosition(), state.angle.getRadians());


		driveMotor.setVoltage(driveOutput + drivefeed);
		turningMotor.setVoltage(turnOutput + turnFeed);
	}

	@Override
	public void periodic() {

	}
}