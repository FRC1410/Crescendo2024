package org.frc1410.chargedup2023.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.frc1410.chargedup2023.util.NetworkTables;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;
import edu.wpi.first.math.geometry.Translation2d;

import static org.frc1410.chargedup2023.util.IDs.*;
import static org.frc1410.chargedup2023.util.Constants.*;
import static edu.wpi.first.wpilibj.SerialPort.Port.kUSB;

public class Drivetrain implements TickedSubsystem {

	// Networktables
	private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Drivetrain");
	private final DoublePublisher xPub = NetworkTables.PublisherFactory(table, "X", 0);
	private final DoublePublisher yPub = NetworkTables.PublisherFactory(table, "Y", 0);
	private final DoublePublisher yaw = NetworkTables.PublisherFactory(table, "yaw", 0);
	private final DoublePublisher frontLeftModulePub = NetworkTables.PublisherFactory(table, "Front left module", 0);
	private final DoublePublisher frontRightModulePub = NetworkTables.PublisherFactory(table, "Front right module", 0);
	private final DoublePublisher backLeftModulePub = NetworkTables.PublisherFactory(table, "Back left module", 0);
	private final DoublePublisher backRightModulePub = NetworkTables.PublisherFactory(table, "Back right module", 0);
	private final DoublePublisher frontLeftSpeed = NetworkTables.PublisherFactory(table, "Front left speed", 0);
	private final DoublePublisher frontRightSpeed = NetworkTables.PublisherFactory(table, "Front right speed", 0);
	private final DoublePublisher backLeftSpeed = NetworkTables.PublisherFactory(table, "Back left speed", 0);
	private final DoublePublisher backRightSpeed = NetworkTables.PublisherFactory(table, "Back right speed", 0);

	private final DoublePublisher frontLeftEncoderValue = NetworkTables.PublisherFactory(table, "Front left encoder value", 0);
	private final DoublePublisher frontRightEncoderValue = NetworkTables.PublisherFactory(table, "Front right encoder value", 0);
	private final DoublePublisher backLeftEncoderValue = NetworkTables.PublisherFactory(table, "Back left encoder value", 0);
	private final DoublePublisher backRightEncoderValue = NetworkTables.PublisherFactory(table, "Back right encoder value", 0);

	private final DoublePublisher frontLeftDesiredStateAngle = NetworkTables.PublisherFactory(table, "Front left desired angle", 0);
	private final DoublePublisher frontRightDesiredStateAngle = NetworkTables.PublisherFactory(table, "Front right desired angle", 0);
	private final DoublePublisher backLeftDesiredStateAngle = NetworkTables.PublisherFactory(table, "Back left desired angle", 0);
	private final DoublePublisher backRightDesiredStateAngle = NetworkTables.PublisherFactory(table, "Back right desired angle", 0);

	//Position from center of the Chassis
	private final Translation2d frontLeftLocation = new Translation2d(-0.263525, 0.263525);
	private final Translation2d frontRightLocation = new Translation2d(0.263525, 0.263525);
	private final Translation2d backLeftLocation = new Translation2d(-0.263525, -0.263525);
	private final Translation2d backRightLocation = new Translation2d(0.263525, -0.263525);

	// Swerve modules
	private final SwerveModule frontLeft = new SwerveModule(FRONT_LEFT_DRIVE_MOTOR, FRONT_LEFT_STEER_MOTOR, FRONT_LEFT_STEER_ENCODER, true);
	private final SwerveModule frontRight = new SwerveModule(FRONT_RIGHT_DRIVE_MOTOR, FRONT_RIGHT_STEER_MOTOR, FRONT_RIGHT_STEER_ENCODER, true);
	private final SwerveModule backLeft = new SwerveModule(BACK_LEFT_DRIVE_MOTOR, BACK_LEFT_STEER_MOTOR, BACK_LEFT_STEER_ENCODER, true);
	private final SwerveModule backRight = new SwerveModule(BACK_RIGHT_DRIVE_MOTOR, BACK_RIGHT_STEER_MOTOR, BACK_RIGHT_STEER_ENCODER, true);

	private final AHRS gyro = new AHRS(kUSB);

	private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
	);

	private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
			kinematics,
			gyro.getRotation2d(),
			new SwerveModulePosition[] {
					frontLeft.getPosition(),
					frontRight.getPosition(),
					backLeft.getPosition(),
					backRight.getPosition()
			});

	public boolean isLocked = false;

	public Drivetrain() {
		backLeft.setEncoderOffset(BL_ANGLE_OFFSET);
		backRight.setEncoderOffset(BR_ANGLE_OFFSET);
		frontLeft.setEncoderOffset(FL_ANGLE_OFFSET);
		frontRight.setEncoderOffset(FR_ANGLE_OFFSET);

		gyro.reset();
	}

	public void drive(double speed, double strafe, double rotation, boolean isFieldRelative) {
		if (speed != 0) {
//			System.out.println("SPEED: " + speed);
		}
		if (isLocked) {
			frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
			frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
			backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(225)));
			backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)));
		} else {
//			System.out.println("NOT LOCKED");
			var swerveModuleStates =
				kinematics.toSwerveModuleStates(
					isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(speed, strafe, rotation, gyro.getRotation2d())
						: new ChassisSpeeds(speed, strafe, rotation));

			SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);
//			System.out.println("Swerve Module State 0 Speed: " + swerveModuleStates[0].speedMetersPerSecond);
//			System.out.println("Swerve Module State 0 Angle: " + swerveModuleStates[0].angle);
			frontLeft.setDesiredState(swerveModuleStates[0]);
			frontRight.setDesiredState(swerveModuleStates[1]);
			backLeft.setDesiredState(swerveModuleStates[2]);
			backRight.setDesiredState(swerveModuleStates[3]);
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
				}
		);
	}

	public void setCoastMode() {
		frontLeft.setDriveCoast();
		frontRight.setDriveCoast();
		backLeft.setDriveCoast();
		backRight.setDriveCoast();
	}

	public void setBreakMode() {
		frontLeft.setDriveBreak();
		frontRight.setDriveBreak();
		backLeft.setDriveBreak();
		backRight.setDriveBreak();
	}

	@Override
	public void periodic() {
		updateOdometry();

		// Networktables
		yaw.set(gyro.getYaw());

		xPub.set(odometry.getPoseMeters().getX());
		yPub.set(odometry.getPoseMeters().getY());

		frontLeft.quoteUnquotePeriodic();
		frontRight.quoteUnquotePeriodic();
		backLeft.quoteUnquotePeriodic();
		backRight.quoteUnquotePeriodic();

		reportEncoderValues();
	}

	public void reportEncoderValues() {
		frontLeftModulePub.set(frontLeft.getDrivePosition());
		frontRightModulePub.set(frontRight.getDrivePosition());
		backLeftModulePub.set(backLeft.getDrivePosition());
		backRightModulePub.set(backRight.getDrivePosition());

		frontLeftSpeed.set(frontLeft.getDriveVel());
		frontRightSpeed.set(frontRight.getDriveVel());
		backLeftSpeed.set(backLeft.getDriveVel());
		backRightSpeed.set(backRight.getDriveVel());

		frontLeftEncoderValue.set(frontLeft.getEncoderValue());
		frontRightEncoderValue.set(frontRight.getEncoderValue());
		backLeftEncoderValue.set(backLeft.getEncoderValue());
		backRightEncoderValue.set(backRight.getEncoderValue());

		frontLeftDesiredStateAngle.set(frontLeft.desiredState.angle.getDegrees());
		frontRightDesiredStateAngle.set(frontRight.desiredState.angle.getDegrees());
		backLeftDesiredStateAngle.set(backLeft.desiredState.angle.getDegrees());
		backRightDesiredStateAngle.set(backRight.desiredState.angle.getDegrees());
	}

	public void runSteer() {
		// done: backRight,
		frontRight.runSteeringMotor();
	}
}
