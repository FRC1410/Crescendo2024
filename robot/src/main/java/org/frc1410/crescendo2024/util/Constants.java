package org.frc1410.crescendo2024.util;

import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import static edu.wpi.first.math.util.Units.degreesToRadians;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;
import java.util.List;

import static org.frc1410.crescendo2024.util.Tuning.*;

public interface Constants {
	// Physical robot constatns
	double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);

	Measure<Distance> WHEEL_RADIUS = Inches.of(2);
	Measure<Distance> WHEEL_CIRCUMFERENCE = WHEEL_RADIUS.times(2 * Math.PI);

	Translation2d FRONT_LEFT_SWERVE_MODULE_LOCATION = new Translation2d(0.2985, 0.2985);
	Translation2d FRONT_RIGHT_SWERVE_MODULE_LOCATION = new Translation2d(0.2985, -0.2985);
	Translation2d BACK_LEFT_SWERVE_MODULE_LOCATION = new Translation2d(-0.2985, 0.2985);
	Translation2d BACK_RIGHT_SWERVE_MODULE_LOCATION = new Translation2d(-0.2985, -0.2985);

	SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
		FRONT_LEFT_SWERVE_MODULE_LOCATION,
		FRONT_RIGHT_SWERVE_MODULE_LOCATION,
		BACK_LEFT_SWERVE_MODULE_LOCATION,
		BACK_RIGHT_SWERVE_MODULE_LOCATION
	);

	// TODO: units
	Measure<Angle> INTAKE_BAR_ENCODER_RANGE = Degrees.of(114);

	Transform3d CAMERA_POSE = new Transform3d(new Translation3d(0.3683,0,0.559), new Rotation3d(0, degreesToRadians(-27), 0));

	// TODO: new value: 5.5m/s
	Measure<Velocity<Distance>> SWERVE_DRIVE_MAX_SPEED = MetersPerSecond.of(4.2);
	Measure<Velocity<Angle>> SWERVE_DRIVE_MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(570);

	PathConstraints PATH_FOLLOWING_CONSTRAINTS = new PathConstraints(
		2.2,
		2.0,
		degreesToRadians(150), 
		degreesToRadians(150));

	HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWING_CONFIG = new HolonomicPathFollowerConfig(
		PATH_FOLLOWING_TRANSLATION_CONSTANTS,
		PATH_FOLLOWING_ROTATION_CONSTANTS,
		3,
		0.37268062,
		new ReplanningConfig(true, true)
	);

	HolonomicPathFollowerConfig HOLONOMIC_AUTO_CONFIG = new HolonomicPathFollowerConfig(
		AUTO_TRANSLATION_CONSTANTS,
		AUTO_ROTATION_CONSTANTS,
		4,
		0.426562165692177,
		new ReplanningConfig()
	);

	Measure<Velocity<Angle>> MAX_SHOOTER_VELOCITY = RPM.of(5800);

	// Speeds
	double INTAKE_SPEED = 0.75;
	double OUTTAKE_SPEED = 0.75;

	Measure<Velocity<Angle>> STORAGE_INTAKE_VELOCITY = RPM.of(300);
	Measure<Velocity<Angle>> STORAGE_OUTTAKE_VELOCITY = RPM.of(400);

	Measure<Velocity<Angle>> SHOOTER_OUTTAKE_VELOCITY = RPM.of(500);

	double INTAKE_BAR_SPEED_DOWN = 0.7;
	double INTAKE_BAR_SPEED_UP = 1;

	Measure<Velocity<Angle>> AUTO_SPEAKER_SHOOTER_VELOCITY = RPM.of(3300);
	Measure<Velocity<Angle>> AUTO_SPEAKER_STORAGE_VELOCITY = RPM.of(700);

	Measure<Velocity<Angle>> MANUAL_SHOOTER_VELOCITY = RPM.of(2400);
	Measure<Velocity<Angle>> MANUAL_STORAGE_VELOCITY = RPM.of(575);
	double MANUAL_INTAKE_SPEED = 0.75;

	Measure<Velocity<Angle>> APM_SHOOTER_VELOCITY = RPM.of(450);

	// Timings
	Measure<Time> SHOOTING_TIME = Seconds.of(0.3);

	// Offsets / inversions
	Measure<Angle> FRONT_LEFT_STEER_ENCODER_OFFSET = Degrees.of(0);
	Measure<Angle> FRONT_RIGHT_STEER_ENCODER_OFFSET = Degrees.of(0);
	Measure<Angle> BACK_LEFT_STEER_ENCODER_OFFSET = Degrees.of(0);
	Measure<Angle> BACK_RIGHT_STEER_ENCODER_OFFSET = Degrees.of(0);

	boolean FRONT_LEFT_DRIVE_MOTOR_INVERTED = true;
	boolean FRONT_LEFT_STEER_MOTOR_INVERTED = true;
	boolean FRONT_RIGHT_DRIVE_MOTOR_INVERTED = true;
	boolean FRONT_RIGHT_STEER_MOTOR_INVERTED = true;
	boolean BACK_LEFT_DRIVE_MOTOR_INVERTED = false;
	boolean BACK_LEFT_STEER_MOTOR_INVERTED = false;
	boolean BACK_RIGHT_DRIVE_MOTOR_INVERTED = true;
	boolean BACK_RIGHT_STEER_MOTOR_INVERTED = true;

	boolean STORAGE_LEFT_MOTOR_INVERTED = false;
	boolean STORAGE_RIGHT_MOTOR_INVERTED = true;

	boolean INTAKE_FRONT_MOTOR_INVERTED = false;
	boolean INTAKE_BACK_MOTOR_INVERTED = false;

	boolean INTAKE_BAR_MOTOR_INVERTED = true;
	boolean INTAKE_EXTENDED_MOTOR_INVERTED = false;

	boolean SHOOTER_LEFT_MOTOR_INVERTED = false;
	boolean SHOOTER_RIGHT_MOTOR_INVERTED = true;

	// Field
	List<ShootingPosition> SHOOTING_POSITIONS_BLUE = Arrays.asList(
		new ShootingPosition(new Pose2d(1.12, 6.76, Rotation2d.fromDegrees(-133)), RPM.of(1850), RPM.of(575)),
		new ShootingPosition(new Pose2d(1.12, 4.34, Rotation2d.fromDegrees(137)), RPM.of(1850), RPM.of(575))
	);

	// List<ShootingPosition> SHOOTING_POSITIONS_RED = Arrays.asList(
	// 	new ShootingPosition(new Pose2d(FIELD_LENGTH - 1.12, 6.76, Rotation2d.fromDegrees(-43)), 1850, 575),
	// 	new ShootingPosition(new Pose2d(FIELD_LENGTH - 1.12, 4.34, Rotation2d.fromDegrees(43)), 1850, 575)
	// );

	List<ShootingPosition> SHOOTING_POSITIONS_RED = SHOOTING_POSITIONS_BLUE
		.stream()
		.map((position) ->
			new ShootingPosition(
				GeometryUtil.flipFieldPose(position.pose), 
				position.shooterVelocity, 
				position.storageVelocity
			)
		)
		.toList();

	Pose2d AMP_SCORING_POSITION_BLUE = new Pose2d(1.83, 7.82, Rotation2d.fromDegrees(90));
	Pose2d AMP_SCORING_POSITION_RED = GeometryUtil.flipFieldPose(AMP_SCORING_POSITION_BLUE);

	// Camera
	String CAMERA_NAME = "Arducam_OV9281_USB_Camera";
  
	// LEDs
	double LED_BRIGHTNESS = 1.0;
	int NUM_LEDS = 28;

	// Shooter
	Measure<Velocity<Angle>> STARTING_SHOOTER_VELOCITY_ADJUSTMENT = RPM.of(300);
	Measure<Velocity<Angle>> SHOOTER_VELOCITY_ADJUSTMENT_MAGNITUDE = RPM.of(150);

	// Feedforward
	double DRIVE_MOTOR_KS = 0.45245;
	double DRIVE_MOTOR_KV = 2.51455;

	double SHOOTER_LEFT_S = 0.07484;
	double SHOOTER_LEFT_V = 0.00207;

	double SHOOTER_RIGHT_S = 0.05369;
	double SHOOTER_RIGHT_V = 0.00210;

	double STORAGE_RIGHT_S = 0.15910;
	double STORAGE_RIGHT_V = 0.00110;

	double STORAGE_LEFT_S = 0.15384;
	double STORAGE_LEFT_V = 0.00107;
}