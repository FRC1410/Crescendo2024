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
import static edu.wpi.first.math.util.Units.degreesToRadians;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.frc1410.crescendo2024.util.Tuning.*;

public final class Constants {
	// speaker april tag 14 trap position
	// x: 24.905972
	// y: 4.266160

	private Constants() {}

	// Physical constants
	public static final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);

	public static final double STORAGE_GEAR_RATIO = 12;

	public static final Measure<Distance> WHEEL_RADIUS = Inches.of(2);
	public static final Measure<Distance> WHEEL_CIRCUMFERENCE = WHEEL_RADIUS.times(2 * Math.PI);

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

	public static final Measure<Angle> INTAKE_BAR_ENCODER_RANGE = Degrees.of(114);

	public static final Transform3d CAMERA_POSE = new Transform3d(new Translation3d(0.3683,0,0.559), new Rotation3d(0, degreesToRadians(-27), 0));

	public static final Measure<Velocity<Distance>> SWERVE_DRIVE_MAX_SPEED = MetersPerSecond.of(5.5);
	public static final Measure<Velocity<Angle>> SWERVE_DRIVE_MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(570);

	// TODO: Speed up
	public static final PathConstraints PATH_FOLLOWING_CONSTRAINTS = new PathConstraints(
		5,
		4,
		degreesToRadians(300), 
		degreesToRadians(400));

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
		SWERVE_DRIVE_MAX_SPEED.in(MetersPerSecond),
		DRIVE_BASE_RADIUS,
		new ReplanningConfig()
	);

	public static final Measure<Velocity<Angle>> MAX_SHOOTER_VELOCITY = RPM.of(5800);

	// Speeds
	public static final double INTAKE_SPEED = 0.75;
	public static final double OUTTAKE_SPEED = 0.75;

	public static final Measure<Velocity<Angle>> STORAGE_INTAKE_VELOCITY = RPM.of(300);
	public static final Measure<Velocity<Angle>> STORAGE_OUTTAKE_VELOCITY = RPM.of(400);

	public static final Measure<Velocity<Angle>> SHOOTER_OUTTAKE_VELOCITY = RPM.of(500);

	public static final double INTAKE_BAR_SPEED_DOWN = 0.7;
	public static final double INTAKE_BAR_SPEED_UP = 1;

	public static final Measure<Velocity<Angle>> AUTO_SPEAKER_SHOOTER_VELOCITY = RPM.of(3900);
	public static final Measure<Velocity<Angle>> AUTO_SPEAKER_SHOOTER_HIGH_VELOCITY = RPM.of(5800);
	public static final Measure<Velocity<Angle>> AUTO_SPEAKER_STORAGE_VELOCITY = RPM.of(700);

	public static final Measure<Velocity<Angle>> SPEAKER_SHOOTER_VELOCITY = RPM.of(2400); 
	public static final Measure<Velocity<Angle>> SPEAKER_STORAGE_VELOCITY = RPM.of(575);
	public static final double SPEAKER_INTAKE_SPEED = 0.75;

	public static final Measure<Velocity<Angle>> AMP_SHOOTER_VELOCITY = RPM.of(480);

	public static final Measure<Velocity<Angle>> SHOOTER_PLOP_VELOCITY = RPM.of(700);

	// Timings
	public static final Measure<Time> SHOOTING_TIME = Seconds.of(0.3);

	// Offsets / inversions
	public static final Measure<Angle> FRONT_LEFT_STEER_ENCODER_OFFSET = Degrees.of(-12.041016 + 90);
	public static final Measure<Angle> FRONT_RIGHT_STEER_ENCODER_OFFSET = Degrees.of(88.154297 - 90);
	public static final Measure<Angle> BACK_LEFT_STEER_ENCODER_OFFSET = Degrees.of(116.279297 - 90);
	public static final Measure<Angle> BACK_RIGHT_STEER_ENCODER_OFFSET = Degrees.of(18.984375 + 90);

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

	public static final boolean INTAKE_SUSHI_ROLLER_MOTOR_INVERTED = false;
	public static final boolean INTAKE_INNER_MOTOR_INVERTED = false;

	public static final boolean INTAKE_BAR_MOTOR_INVERTED = true;
	public static final boolean INTAKE_OUTER_MOTOR_INVERTED = false;

	public static final boolean SHOOTER_LEFT_MOTOR_INVERTED = false;
	public static final boolean SHOOTER_RIGHT_MOTOR_INVERTED = true;

	// Field
	public static final List<ShootingPosition> SHOOTING_POSITIONS_BLUE = List.of(
		new ShootingPosition(new Pose2d(1.12, 6.76, Rotation2d.fromDegrees(-133)), RPM.of(1850), RPM.of(575)),
		new ShootingPosition(new Pose2d(1.12, 4.34, Rotation2d.fromDegrees(137)), RPM.of(1850), RPM.of(575))
	);

	public static final List<ShootingPosition> SHOOTING_POSITIONS_RED = SHOOTING_POSITIONS_BLUE
		.stream()
		.map((position) ->
			new ShootingPosition(
				GeometryUtil.flipFieldPose(position.pose), 
				position.shooterVelocity, 
				position.storageVelocity
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
			System.out.println("Failed to load april tag field layout");
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
	public static final double MAX_APRIL_TAG_DISTANCE = 4;
  
	// LEDs
	public static final double LED_BRIGHTNESS = 1.0;
	public static final int NUM_LEDS = 28;

	// Shooter
	public static final Measure<Velocity<Angle>> STARTING_SHOOTER_VELOCITY_ADJUSTMENT = RPM.of(600);
	public static final Measure<Velocity<Angle>> SHOOTER_VELOCITY_ADJUSTMENT_MAGNITUDE = RPM.of(150);

	// Feedforward
	public static final double DRIVE_KS = 0.31720;
	public static final double DRIVE_KV = 0.12280;

	public static final double SHOOTER_LEFT_S = 0.07484;
	public static final double SHOOTER_LEFT_V = 0.00207;

	public static final double SHOOTER_RIGHT_S = 0.05369;
	public static final double SHOOTER_RIGHT_V = 0.00210;

	public static final double STORAGE_RIGHT_S = 0.15910;
	public static final double STORAGE_RIGHT_V = 0.00110;

	public static final double STORAGE_LEFT_S = 0.15384;
	public static final double STORAGE_LEFT_V = 0.00107;
}