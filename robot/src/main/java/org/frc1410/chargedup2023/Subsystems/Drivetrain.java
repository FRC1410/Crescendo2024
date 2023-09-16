package org.frc1410.chargedup2023.Subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;

import org.frc1410.framework.scheduler.subsystem.SubsystemStore;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

import static org.frc1410.chargedup2023.util.IDs.*;

import org.frc1410.chargedup2023.util.NetworkTables;

import static org.frc1410.chargedup2023.util.Constants.*;

public class Drivetrain implements TickedSubsystem {
	private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Drivetrain");

	private final DoublePublisher chassisSpeedsX = NetworkTables.PublisherFactory(table, "ChassisSpeeds X", 0);
	private final DoublePublisher chassisSpeedsY = NetworkTables.PublisherFactory(table, "ChassisSpeeds Y", 0);
	private final DoublePublisher chassisSpeedsRotation = NetworkTables.PublisherFactory(table,
			"ChassisSpeeds Rotation", 0);

	private final DoublePublisher frontLeftVoltage = NetworkTables.PublisherFactory(table, "frontLeftVoltage", 0);
	private final DoublePublisher frontRightVoltage = NetworkTables.PublisherFactory(table, "frontRightVoltage", 0);
	private final DoublePublisher backLeftVoltage = NetworkTables.PublisherFactory(table, "backLeftVoltage", 0);
	private final DoublePublisher backRightVoltage = NetworkTables.PublisherFactory(table, "backRightVoltage", 0);

	private final DoublePublisher navXYaw = NetworkTables.PublisherFactory(table, "NavX Yaw", 0);

	private final SwerveModule frontLeft;
	private final SwerveModule frontRight;
	private final SwerveModule backLeft;
	private final SwerveModule backRight;

	private final AHRS gyro = new AHRS(SPI.Port.kMXP);

	private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			FRONT_LEFT_SWERVE_MODULE_LOCATION,
			FRONT_RIGHT_SWERVE_MODULE_LOCATION,
			BACK_LEFT_SWERVE_MODULE_LOCATION,
			BACK_RIGHT_SWERVE_MODULE_LOCATION);

	private final SwerveDriveOdometry odometry;

	public boolean isLocked = false;

	public Drivetrain(SubsystemStore subsystems) {
		this.frontLeft = subsystems.track(new SwerveModule(FRONT_LEFT_DRIVE_MOTOR, FRONT_LEFT_STEER_MOTOR,
				FRONT_LEFT_STEER_ENCODER, false, true, FRONT_LEFT_STEER_ENCODER_OFFSET, frontLeftVoltage));
		this.frontRight = subsystems.track(new SwerveModule(FRONT_RIGHT_DRIVE_MOTOR, FRONT_RIGHT_STEER_MOTOR,
				FRONT_RIGHT_STEER_ENCODER, false, true, FRONT_RIGHT_STEER_ENCODER_OFFSET, frontRightVoltage));
		this.backLeft = subsystems.track(new SwerveModule(BACK_LEFT_DRIVE_MOTOR, BACK_LEFT_STEER_MOTOR,
				BACK_LEFT_STEER_ENCODER, true, true, BACK_LEFT_STEER_ENCODER_OFFSET, backLeftVoltage));
		this.backRight = subsystems.track(new SwerveModule(BACK_RIGHT_DRIVE_MOTOR, BACK_RIGHT_STEER_MOTOR,
				BACK_RIGHT_STEER_ENCODER, false, true, BACK_RIGHT_STEER_ENCODER_OFFSET, backRightVoltage));

		this.odometry = new SwerveDriveOdometry(
				kinematics,
				gyro.getRotation2d(),
				new SwerveModulePosition[] {
						frontLeft.getPosition(),
						frontRight.getPosition(),
						backLeft.getPosition(),
						backRight.getPosition()
				});

		gyro.reset();

		gyro.calibrate();
	}

	// private float yawOffset;

	public void zeroYaw() {
		// System.out.println("zero Yaw");
		this.gyro.zeroYaw();
		// yawOffset = this.gyro.getRotation2d()
	}

	public void zero() {
		frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
		frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
		backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
		backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
	}

	public static Twist2d log(final Pose2d transform) {
		double kEps = 1E-9;
		final double dtheta = transform.getRotation().getRadians();
		final double half_dtheta = 0.5 * dtheta;
		final double cos_minus_one = Math.cos(transform.getRotation().getRadians()) - 1.0;
		double halftheta_by_tan_of_halfdtheta;
		if (Math.abs(cos_minus_one) < kEps) {
		  halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
		} else {
		  halftheta_by_tan_of_halfdtheta =
			  -(half_dtheta * Math.sin(transform.getRotation().getRadians())) / cos_minus_one;
		}
		final Translation2d translation_part =
			transform
				.getTranslation()
				.rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
		return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
	  }

	private ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
		final double LOOP_TIME_S = 0.02;
		Pose2d futureRobotPose = new Pose2d(
				originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
				originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
				Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
		Twist2d twistForPose = log(futureRobotPose);
		ChassisSpeeds updatedSpeeds = new ChassisSpeeds(
				twistForPose.dx / LOOP_TIME_S,
				twistForPose.dy / LOOP_TIME_S,
				twistForPose.dtheta / LOOP_TIME_S);
		return updatedSpeeds;
	}

	public void drive(double xVelocity, double yVelocity, double rotation, boolean isFieldRelative) {
		navXYaw.set(gyro.getYaw());
		if (isLocked) {
			frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
			frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
			backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
			backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		} else {
			chassisSpeedsX.set(xVelocity);
			chassisSpeedsY.set(yVelocity);
			chassisSpeedsRotation.set(rotation);

			var chassisSpeeds = isFieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotation, gyro.getRotation2d())
				: new ChassisSpeeds(xVelocity, yVelocity, rotation);

			var swerveModuleStates = kinematics.toSwerveModuleStates(correctForDynamics(chassisSpeeds));

			SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

			frontLeft.setDesiredState(swerveModuleStates[1]);
			frontRight.setDesiredState(swerveModuleStates[3]);
			backLeft.setDesiredState(swerveModuleStates[0]);
			backRight.setDesiredState(swerveModuleStates[2]);
		}
	}

	public void updateOdometry() {
		odometry.update(
				gyro.getRotation2d(),
				new SwerveModulePosition[] {
						frontLeft.getPosition(),
						frontRight.getPosition(),
						backLeft.getPosition(),
						backRight.getPosition()
				});
	}

	@Override
	public void periodic() {
		updateOdometry();
	}
}
