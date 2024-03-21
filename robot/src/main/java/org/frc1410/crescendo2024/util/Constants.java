package org.frc1410.crescendo2024.util;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import java.util.Arrays;
import java.util.List;

import static org.frc1410.crescendo2024.util.Tuning.*;

public interface Constants {
	// Physical constatns
	double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);

	double WHEEL_RADIUS = 0.0508;
	double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;

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

	int INTAKE_BAR_ENCODER_RANGE = 648;

	Transform3d CAMERA_POSE = new Transform3d(new Translation3d(0.3683,0,0.559), new Rotation3d(0,Units.degreesToRadians(-27),0));

	// Constraints
	double DRIVE_MOTOR_FREE_SPEED_RPM = 5676;
	double DRIVE_MOTOR_FREE_SPEED_RPS = DRIVE_MOTOR_FREE_SPEED_RPM / 60;
	double DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND = ((DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE) / DRIVE_GEAR_RATIO);

	double SWERVE_DRIVE_MAX_SPEED = 4.2;
	double SWERVE_DRIVE_MAX_ANGULAR_VELOCITY = 10;

	PathConstraints PATH_FOLLOWING_CONSTRAINTS = new PathConstraints(
		2.2,
		2.0,
		Units.degreesToRadians(150), 
		Units.degreesToRadians(150));

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

	double MAX_SHOOTER_RPM = 5800;

	// Speeds
	double INTAKE_SPEED = 0.75;
	double OUTTAKE_SPEED = 0.75;

	double STORAGE_INTAKE_RPM = 300;
	double STORAGE_OUTTAKE_RPM = 400;

	double SHOOTER_OUTTAKE_RPM = 500;

	double INTAKE_BAR_SPEED_DOWN = 0.7;
	double INTAKE_BAR_SPEED_UP = 1;

	double AUTO_SPEAKER_SHOOTER_RPM = 3300;
	double AUTO_SPEAKER_STORAGE_RPM = 700;

	double MANUAL_SHOOTER_RPM = 2400;
	double MANUAL_STORAGE_RPM = 575;
	double MANUAL_INTAKE_SPEED = 0.75;

	// Timings
	double SHOOTING_TIME = 0.3;

	// Offsets / inversions
	double FRONT_LEFT_STEER_ENCODER_OFFSET = -1.549321;
	double FRONT_RIGHT_STEER_ENCODER_OFFSET = -0.207087;
	double BACK_LEFT_STEER_ENCODER_OFFSET = 0.520019;
	double BACK_RIGHT_STEER_ENCODER_OFFSET = 1.869923;

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
	double FIELD_LENGTH = 16.54;

	List<ShootingPosition> SHOOTING_POSITIONS_BLUE = Arrays.asList(
		new ShootingPosition(new Pose2d(1.12, 6.76, Rotation2d.fromDegrees(-133)), 1850, 575),
		new ShootingPosition(new Pose2d(1.12, 4.34, Rotation2d.fromDegrees(137)), 1850, 575)
	);

	List<ShootingPosition> SHOOTING_POSITIONS_RED = Arrays.asList(
		new ShootingPosition(new Pose2d(FIELD_LENGTH - 1.12, 6.76, Rotation2d.fromDegrees(-43)), 1850, 575),
		new ShootingPosition(new Pose2d(FIELD_LENGTH - 1.12, 4.34, Rotation2d.fromDegrees(43)), 1850, 575)
	);

	// Camera
	String CAMERA_NAME = "Arducam_OV9281_USB_Camera";
  
	// LEDs
	double LED_BRIGHTNESS = 1.0;
	int NUM_LEDS = 250;

	// Shooter
	double STARTING_SHOOTER_RPM_ADJUSTMENT = 0;
	double SHOOTER_RPM_ADJUSTMENT_MAGNITUDE = 150;

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