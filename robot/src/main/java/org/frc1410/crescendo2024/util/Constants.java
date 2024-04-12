package org.frc1410.crescendo2024.util;

import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.frc1410.crescendo2024.util.Tuning.*;

public final class Constants {
	private Constants() {}

	// Physical constatns
	public static final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);

	public static final double WHEEL_RADIUS = 0.0508;
	public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;

	public static final Translation2d FRONT_LEFT_SWERVE_MODULE_LOCATION = new Translation2d(0.301625, .301625);
	public static final Translation2d FRONT_RIGHT_SWERVE_MODULE_LOCATION = new Translation2d(0.301625, -0.301625);
	public static final Translation2d BACK_LEFT_SWERVE_MODULE_LOCATION = new Translation2d(-0.301625, 0.301625);
	public static final Translation2d BACK_RIGHT_SWERVE_MODULE_LOCATION = new Translation2d(-0.301625, -0.301625);

	public static final double DRIVE_BASE_RADIUS = 0.4265621658;

	public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
		FRONT_LEFT_SWERVE_MODULE_LOCATION,
		FRONT_RIGHT_SWERVE_MODULE_LOCATION,
		BACK_LEFT_SWERVE_MODULE_LOCATION,
		BACK_RIGHT_SWERVE_MODULE_LOCATION
	);

	public static final int INTAKE_BAR_ENCODER_RANGE = 648;

	public static final Transform3d CAMERA_POSE = new Transform3d(new Translation3d(0.3683,0,0.559), new Rotation3d(0, Units.degreesToRadians(-27), 0));

	// Constraints
	public static final double DRIVE_MOTOR_FREE_SPEED_RPM = 5676;
	public static final double DRIVE_MOTOR_FREE_SPEED_RPS = DRIVE_MOTOR_FREE_SPEED_RPM / 60;
	public static final double DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND = ((DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE) / DRIVE_GEAR_RATIO);

	// TODO: new value: 5.5m/s
	public static final double SWERVE_DRIVE_MAX_SPEED = 5.2;
	public static final double SWERVE_DRIVE_MAX_ANGULAR_VELOCITY = 10;

	public static final PathConstraints PATH_FOLLOWING_CONSTRAINTS = new PathConstraints(
		2.2,
		2.0,
		Units.degreesToRadians(150), 
		Units.degreesToRadians(150));

	public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWING_CONFIG = new HolonomicPathFollowerConfig(
		PATH_FOLLOWING_TRANSLATION_CONSTANTS,
		PATH_FOLLOWING_ROTATION_CONSTANTS,
		3,
		DRIVE_BASE_RADIUS,
		new ReplanningConfig(true, true)
	);

	public static final HolonomicPathFollowerConfig HOLONOMIC_AUTO_CONFIG = new HolonomicPathFollowerConfig(
		AUTO_TRANSLATION_CONSTANTS,
		AUTO_ROTATION_CONSTANTS,
		5.5,
		DRIVE_BASE_RADIUS,
		new ReplanningConfig()
	);

	public static final double MAX_SHOOTER_RPM = 5800;

	// Speeds
	public static final double INTAKE_SPEED = 0.75;
	public static final double OUTTAKE_SPEED = 0.75;

	public static final double STORAGE_INTAKE_RPM = 300;
	public static final double STORAGE_OUTTAKE_RPM = 400;

	public static final double SHOOTER_OUTTAKE_RPM = 500;

	public static final double INTAKE_BAR_SPEED_DOWN = 0.7;
	public static final double INTAKE_BAR_SPEED_UP = 1;

	public static final double AUTO_SPEAKER_SHOOTER_RPM = 3300;
	public static final double AUTO_SPEAKER_STORAGE_RPM = 700;

	public static final double MANUAL_SHOOTER_RPM = 2400; 
	public static final double MANUAL_STORAGE_RPM = 575;
	public static final double MANUAL_INTAKE_SPEED = 0.75;

	public static final double APM_SHOOTER_RPM = 450;

	public static final double SHOOTER_PLOP_RPM = 700;

	// Timings
	public static final double SHOOTING_TIME = 0.3;

	// Offsets / inversions
	public static final double FRONT_LEFT_STEER_ENCODER_OFFSET = -12.041016 + 90;
	public static final double FRONT_RIGHT_STEER_ENCODER_OFFSET = 88.154297 - 90;
	public static final double BACK_LEFT_STEER_ENCODER_OFFSET = 116.279297 - 90;
	public static final double BACK_RIGHT_STEER_ENCODER_OFFSET = 18.984375 + 90;

	public static final boolean FRONT_LEFT_DRIVE_MOTOR_INVERTED = false;
	public static final boolean FRONT_RIGHT_DRIVE_MOTOR_INVERTED = true;
	public static final boolean BACK_LEFT_DRIVE_MOTOR_INVERTED = false;
	public static final boolean BACK_RIGHT_DRIVE_MOTOR_INVERTED = true;

	public static final boolean FRONT_LEFT_STEER_MOTOR_INVERTED = true;
	public static final boolean FRONT_RIGHT_STEER_MOTOR_INVERTED = true;
	public static final boolean BACK_LEFT_STEER_MOTOR_INVERTED = false;
	public static final boolean BACK_RIGHT_STEER_MOTOR_INVERTED = true;

	public static final boolean STORAGE_LEFT_MOTOR_INVERTED = false;
	public static final boolean STORAGE_RIGHT_MOTOR_INVERTED = true;

	public static final boolean INTAKE_FRONT_MOTOR_INVERTED = false;
	public static final boolean INTAKE_BACK_MOTOR_INVERTED = false;

	public static final boolean INTAKE_BAR_MOTOR_INVERTED = true;
	public static final boolean INTAKE_EXTENDED_MOTOR_INVERTED = false;

	public static final boolean SHOOTER_LEFT_MOTOR_INVERTED = false;
	public static final boolean SHOOTER_RIGHT_MOTOR_INVERTED = true;

	// Field
	public static final double FIELD_LENGTH = 16.54;

	public static final List<ShootingPosition> SHOOTING_POSITIONS_BLUE = List.of(
		new ShootingPosition(new Pose2d(1.12, 6.76, Rotation2d.fromDegrees(-133)), 1850, 575),
		new ShootingPosition(new Pose2d(1.12, 4.34, Rotation2d.fromDegrees(137)), 1850, 575)
	);

	public static final List<ShootingPosition> SHOOTING_POSITIONS_RED = SHOOTING_POSITIONS_BLUE
		.stream()
		.map((position) -> 
			new ShootingPosition(
				GeometryUtil.flipFieldPose(position.pose),
				position.shooterRPM,
				position.storageRPM
			)
		)
		.toList();

	public static final Pose2d AMP_SCORING_POSITION_BLUE = new Pose2d(1.83, 7.82, Rotation2d.fromDegrees(90));
	public static final Pose2d AMP_SCORING_POSITION_RED = GeometryUtil.flipFieldPose(AMP_SCORING_POSITION_BLUE);

	public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

	static {
		AprilTagFieldLayout layout;

		// layout = new AprilTagFieldLayout(List.of(new AprilTag(4, new Pose3d(16.579342,  5.547867999999999, 1.4511020000000001, new Rotation3d(new Quaternion(6.123233995736766e-17, 0, 0, 1))))), 16.541, 8.211);

		try {
			layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
		} catch(IOException e) {
			DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
			layout = null;
		}

		APRIL_TAG_FIELD_LAYOUT = layout;
	}

	public static final Map<String, Pose2d> DEFENSIVE_AUTO_STOPPING_POSITIONS_BLUE = Map.of(
		"4", new Pose2d(7.92, 2, Rotation2d.fromDegrees(-135)),
		"3", new Pose2d(7.92, 3.65, Rotation2d.fromDegrees(-135)),
		"2", new Pose2d(7.92, 5.35, Rotation2d.fromDegrees(-135)),
		"1", new Pose2d(7.92, 7, Rotation2d.fromDegrees(-135))
	);

	public static final Map<String, Pose2d> DEFENSIVE_AUTO_STOPPING_POSITIONS_RED = new HashMap<>();

	static {
		DEFENSIVE_AUTO_STOPPING_POSITIONS_BLUE.forEach((key, value) ->
			DEFENSIVE_AUTO_STOPPING_POSITIONS_RED.put(
				key,
				GeometryUtil.flipFieldPose(value)
			)
		);
	}

	// Camera
	public static final String CAMERA_NAME = "Arducam_OV9281_USB_Camera";

	public static final double MAX_APRIL_TAG_AMBIGUITY = 0.2;
	public static final double MAX_APRIL_TAG_DISTANCE = 3;
  
	// LEDs
	public static final double LED_BRIGHTNESS = 1.0;
	public static final int NUM_LEDS = 28;

	// Shooter
	public static final double STARTING_SHOOTER_RPM_ADJUSTMENT = 300;
	public static final double SHOOTER_RPM_ADJUSTMENT_MAGNITUDE = 150;

	// Feedforward
	public static final double DRIVE_MOTOR_KS = 0.45245;
	public static final double DRIVE_MOTOR_KV = 2.51455;

	public static final double SHOOTER_LEFT_S = 0.07484;
	public static final double SHOOTER_LEFT_V = 0.00207;

	public static final double SHOOTER_RIGHT_S = 0.05369;
	public static final double SHOOTER_RIGHT_V = 0.00210;

	public static final double STORAGE_RIGHT_S = 0.15910;
	public static final double STORAGE_RIGHT_V = 0.00110;

	public static final double STORAGE_LEFT_S = 0.15384;
	public static final double STORAGE_LEFT_V = 0.00107;
}