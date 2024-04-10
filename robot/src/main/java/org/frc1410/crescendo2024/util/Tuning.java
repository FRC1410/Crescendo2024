package org.frc1410.crescendo2024.util;

import com.pathplanner.lib.util.PIDConstants;

public final class Tuning {
	private Tuning() {}

	// Drivetrain
	public static final double SWERVE_DRIVE_P = 0;
	public static final double SWERVE_DRIVE_I = 0;
	public static final double SWERVE_DRIVE_D = 0;
	public static final double SWERVE_DRIVE_FF = 0.00032;

    public static final double SWERVE_STEERING_P = 4;
	public static final double SWERVE_STEERING_I = 0;
	public static final double SWERVE_STEERING_D = 0;

	// Shooter
	public static final double SHOOTER_LEFT_P = 0;
	public static final double SHOOTER_LEFT_I = 0;
	public static final double SHOOTER_LEFT_D = 0;
	public static final double SHOOTER_LEFT_FF = 1;

	public static final double SHOOTER_RIGHT_P = 0;
	public static final double SHOOTER_RIGHT_I = 0;
	public static final double SHOOTER_RIGHT_D = 0;
	public static final double SHOOTER_RIGHT_FF = 1;

	// Auto
	/* PID CONSTANTS

	OLD DRIVE: 5, 0, 0
	OLD TURN: 4, 0, 0

	CURRENT DRIVE: 6.5, 0, 0
	CURRENT TURN: 4.5, 0, 0

	TESTED: P: 4.5, 4(WORKS),  I: D

	WORKS: P (6, 4)

	 */

	public static final PIDConstants AUTO_TRANSLATION_CONSTANTS = new PIDConstants(7.505, 0, 0);
	public static final PIDConstants AUTO_ROTATION_CONSTANTS = new PIDConstants(4.105, 0, 0);

	// Path following
	public static final PIDConstants PATH_FOLLOWING_TRANSLATION_CONSTANTS = new PIDConstants(6,0, 0);
	public static final PIDConstants PATH_FOLLOWING_ROTATION_CONSTANTS = new PIDConstants(2, 0, 0);
}