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


	//Position from center of the Chassis
	private final Translation2d frontLeftLocation = new Translation2d(-0.263525, 0.263525);
	private final Translation2d frontRightLocation = new Translation2d(0.263525, 0.263525);
	private final Translation2d backLeftLocation = new Translation2d(-0.263525, -0.263525);
	private final Translation2d backRightLocation = new Translation2d(0.263525, -0.263525);

	// Swerve modules
	private final SwerveModule frontLeft = new SwerveModule(LEFT_FRONT_DRIVE_MOTOR, LEFT_FRONT_STEER_MOTOR, LEFT_FRONT_STEER_ENCODER);
	private final SwerveModule frontRight = new SwerveModule(RIGHT_FRONT_DRIVE_MOTOR, RIGHT_FRONT_STEER_MOTOR, RIGHT_FRONT_STEER_ENCODER);
	private final SwerveModule backLeft = new SwerveModule(LEFT_BACK_DRIVE_MOTOR, LEFT_BACK_STEER_MOTOR, LEFT_BACK_STEER_ENCODER);
	private final SwerveModule backRight = new SwerveModule(RIGHT_BACK_DRIVE_MOTOR, RIGHT_BACK_STEER_MOTOR, RIGHT_BACK_STEER_ENCODER);

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

	public Drivetrain() {
		gyro.reset();
	}

	public void drive(double speed, double strafe, double rotation, boolean isFieldRelative, boolean isLocked) {
		var swerveModuleStates =
				kinematics.toSwerveModuleStates(
						isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(speed, strafe, rotation, gyro.getRotation2d())
								: new ChassisSpeeds(speed, strafe, rotation));

		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);
		frontLeft.setDesiredState(swerveModuleStates[0]);
		frontRight.setDesiredState(swerveModuleStates[1]);
		backLeft.setDesiredState(swerveModuleStates[2]);
		backRight.setDesiredState(swerveModuleStates[3]);

		if(isLocked = true) {
			frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
			frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
			backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(225)));
			backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)));
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

		frontLeftModulePub.set(frontLeft.getDrivePosition());
		frontRightModulePub.set(frontRight.getDrivePosition());
		backLeftModulePub.set(backLeft.getDrivePosition());
		backRightModulePub.set(backRight.getDrivePosition());

		frontLeftSpeed.set(frontLeft.getDriveVel());
		frontRightSpeed.set(frontRight.getDriveVel());
		backLeftSpeed.set(backLeft.getDriveVel());
		backRightSpeed.set(backRight.getDriveVel());
	}
}