package org.frc1410.chargedup2023.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
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
	private final DoublePublisher chassisSpeedsRotation = NetworkTables.PublisherFactory(table, "ChassisSpeeds Rotation", 0);

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
				FRONT_LEFT_STEER_ENCODER, false, true, FRONT_LEFT_STEER_ENCODER_OFFSET));
		this.frontRight = subsystems.track(new SwerveModule(FRONT_RIGHT_DRIVE_MOTOR, FRONT_RIGHT_STEER_MOTOR,
				FRONT_RIGHT_STEER_ENCODER, false, true, FRONT_RIGHT_STEER_ENCODER_OFFSET));
		this.backLeft = subsystems.track(new SwerveModule(BACK_LEFT_DRIVE_MOTOR, BACK_LEFT_STEER_MOTOR,
				BACK_LEFT_STEER_ENCODER, true, true, BACK_LEFT_STEER_ENCODER_OFFSET));
		this.backRight = subsystems.track(new SwerveModule(BACK_RIGHT_DRIVE_MOTOR, BACK_RIGHT_STEER_MOTOR,
				BACK_RIGHT_STEER_ENCODER, false, true, BACK_RIGHT_STEER_ENCODER_OFFSET));

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
	}

	public void drive(double xVelocity, double yVelocity, double rotation, boolean isFieldRelative) {
		if (isLocked) {
			frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
			frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
			backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
			backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		} else {
			chassisSpeedsX.set(xVelocity);
			chassisSpeedsY.set(yVelocity);
			chassisSpeedsRotation.set(rotation);

			var swerveModuleStates = kinematics.toSwerveModuleStates(
					isFieldRelative
							? ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotation,
									gyro.getRotation2d())
							: new ChassisSpeeds(xVelocity, yVelocity, rotation));

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
