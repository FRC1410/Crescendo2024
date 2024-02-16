package org.frc1410.crescendo2024.util;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation3d;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

import static org.frc1410.crescendo2024.util.Tuning.*;

public interface Constants {
	double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);

	double WHEEL_RADIUS = 0.0508;
	double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;

	double DRIVE_MOTOR_FREE_SPEED_RPM = 5676;
	double DRIVE_MOTOR_FREE_SPEED_RPS = DRIVE_MOTOR_FREE_SPEED_RPM / 60;
	double DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND = ((DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE) / DRIVE_GEAR_RATIO);

	double SWERVE_DRIVE_MAX_SPEED = 4.2;
	double SWERVE_DRIVE_MAX_ANGULAR_VELOCITY = 4;

	double FRONT_LEFT_STEER_ENCODER_OFFSET = -1.549321;
	double FRONT_RIGHT_STEER_ENCODER_OFFSET = 2.293301;
	double BACK_LEFT_STEER_ENCODER_OFFSET = 2.659923;
	double BACK_RIGHT_STEER_ENCODER_OFFSET = -0.852893;

	// TODO: Change to actual values
	Translation2d FRONT_LEFT_SWERVE_MODULE_LOCATION = new Translation2d(0.301625, 0.301625);
	Translation2d FRONT_RIGHT_SWERVE_MODULE_LOCATION = new Translation2d(0.301625, -0.301625);
	Translation2d BACK_LEFT_SWERVE_MODULE_LOCATION = new Translation2d(-0.301625, 0.301625);
	Translation2d BACK_RIGHT_SWERVE_MODULE_LOCATION = new Translation2d(-0.301625, -0.301625);

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
  
	double LED_BRIGHTNESS = 1.0;
	int NUM_LEDS = 250;

	List<ShootingPosition> SHOOTING_POSITIONS = Arrays.asList(
//		new ShootingPosition(new Pose2d(2.26, 4.81, Rotation2d.fromDegrees(-22.6)), 0, 0),
		new ShootingPosition(new Pose2d(1.43, 6.53, Rotation2d.fromDegrees(-155.7)), 2700, 625)
	);

	PathConstraints PATH_FIND_CONSTRAINTS = new PathConstraints(
		3.2, 4.0,
		Units.degreesToRadians(150), Units.degreesToRadians(150));

	HolonomicPathFollowerConfig PATH_FIND_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
		new PIDConstants(5.0,0.0, 0),
		new PIDConstants(1, 0.0, 0),
		3,
		0.37268062,
		new ReplanningConfig()
	);

	Rotation3d NAVX_ANGLE = new Rotation3d(0, 0, 0);

	double INTAKE_SPEED = 0.75;
	double OUTTAKE_SPEED = -0.75;

	double STORAGE_INTAKE_RPM = 300;
	double STORAGE_OUTTAKE_SPEED = -400;

	double SHOOTER_OUTTAKE_SPEED = -500;

	double SHOOTER_RPM_INCREMENT = 0;

	int AMP_HOME_POS = 0;
	double AMP_SHOOT_SPEED = 0;
	double STORAGE_SPEED = 0;

	boolean STORAGE_LEFT_MOTOR_INVERTED = false;
	boolean STORAGE_RIGHT_MOTOR_INVERTED = true;

	boolean AMP_BAR_MOTOR_INVERTED = false;

	boolean INTAKE_FRONT_MOTOR_INVERTED = false;
	boolean INTAKE_BACK_MOTOR_INVERTED = false;

	boolean SHOOTER_LEFT_MOTOR_INVERTED = false;
	boolean SHOOTER_RIGHT_MOTOR_INVERTED = true;

	double SHOOTER_MANUAL_RPM = 1000;

	HolonomicPathFollowerConfig HOLONOMIC_AUTO_CONFIG = new HolonomicPathFollowerConfig(
		new PIDConstants(AUTO_DRIVE_P, AUTO_DRIVE_I, AUTO_DRIVE_D),
		new PIDConstants(AUTO_TURN_P, AUTO_TURN_I, AUTO_TURN_D),
		3,
		0.426562165692177,
		new ReplanningConfig()
	);

	double MAX_SHOOTER_RPM = 5800;

	//	Count=392
	//	R2=0.99966
	// 	kS=0.45245
	// 	kV=2.61455

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

	Transform3d CAMERA_POSE = new Transform3d(new Translation3d(0.3683,0,0.559), new Rotation3d(0,Math.toRadians(16),0));
}