package org.frc1410.crescendo2024.util;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import java.util.Arrays;
import java.util.List;

public interface Constants {
	double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);

	double WHEEL_RADIUS = 0.0508;
	double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;

	double DRIVE_MOTOR_FREE_SPEED_RPM = 5676;
	double DRIVE_MOTOR_FREE_SPEED_RPS = DRIVE_MOTOR_FREE_SPEED_RPM / 60;
	double DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND = ((DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE) / DRIVE_GEAR_RATIO);

	double SWERVE_DRIVE_MAX_SPEED = 4.2;
	double SWERVE_DRIVE_MAX_ANGULAR_VELOCITY = 4;

	double FRONT_LEFT_STEER_ENCODER_OFFSET = 0.173340;
	double FRONT_RIGHT_STEER_ENCODER_OFFSET = 0.497314;
	double BACK_LEFT_STEER_ENCODER_OFFSET = -0.469482;
	double BACK_RIGHT_STEER_ENCODER_OFFSET = -0.300049;

	// TODO: Change to actual values
	Translation2d FRONT_LEFT_SWERVE_MODULE_LOCATION = new Translation2d(0.263525, 0.263525);
	Translation2d FRONT_RIGHT_SWERVE_MODULE_LOCATION = new Translation2d(0.263525, -0.263525);
	Translation2d BACK_LEFT_SWERVE_MODULE_LOCATION = new Translation2d(-0.263525, 0.263525);
	Translation2d BACK_RIGHT_SWERVE_MODULE_LOCATION = new Translation2d(-0.263525, -0.263525);

	SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
		FRONT_LEFT_SWERVE_MODULE_LOCATION,
		FRONT_RIGHT_SWERVE_MODULE_LOCATION,
		BACK_LEFT_SWERVE_MODULE_LOCATION,
		BACK_RIGHT_SWERVE_MODULE_LOCATION
	);

	boolean FRONT_LEFT_DRIVE_MOTOR_INVERTED = true;
	boolean FRONT_LEFT_STEER_MOTOR_INVERTED = true;
	boolean FRONT_RIGHT_DRIVE_MOTOR_INVERTED = false;
	boolean FRONT_RIGHT_STEER_MOTOR_INVERTED = true;
	boolean BACK_LEFT_DRIVE_MOTOR_INVERTED = true;
	boolean BACK_LEFT_STEER_MOTOR_INVERTED = true;
	boolean BACK_RIGHT_DRIVE_MOTOR_INVERTED = false;
	boolean BACK_RIGHT_STEER_MOTOR_INVERTED = true;

	String CAMERA_NAME = "Arducam_OV9281_USB_Camera";
  
	int LED_ID = 29;
	double LED_BRIGHTNESS = 1.0;
	int NUM_LEDS = 250;

	List<ShootingPosition> SHOOTING_POSITIONS = Arrays.asList(
		new ShootingPosition(new Pose2d(2.26, 4.81, Rotation2d.fromDegrees(-22.6)), 0),
		new ShootingPosition(new Pose2d(1.88, 5, Rotation2d.fromDegrees(-19.6)), 0)
	);

	PathConstraints PATH_FIND_CONSTRAINTS = new PathConstraints(
		3.2, 4.0,
		Units.degreesToRadians(150), Units.degreesToRadians(150));

	HolonomicPathFollowerConfig PATH_FIND_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
		new PIDConstants(1.0,0.0, 0.5),
		new PIDConstants(1, 0.0, 0),
		3,
		0.37268062,
		new ReplanningConfig()
	);
}