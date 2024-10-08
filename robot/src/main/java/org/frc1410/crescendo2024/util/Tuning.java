package org.frc1410.crescendo2024.util;

import com.pathplanner.lib.util.PIDConstants;

public final class Tuning {
	private Tuning() {}

	// Drivetrain
	public static final double SWERVE_DRIVE_P = 0;
	public static final double SWERVE_DRIVE_I = 0;
	public static final double SWERVE_DRIVE_D = 0;

    public static final double SWERVE_STEERING_P = 4;
	public static final double SWERVE_STEERING_I = 0;
	public static final double SWERVE_STEERING_D = 0;

	// Shooter
	public static final double SHOOTER_P = 0;
	public static final double SHOOTER_I = 0;
	public static final double SHOOTER_D = 0;

	// Auto
	public static final PIDConstants AUTO_TRANSLATION_CONSTANTS = new PIDConstants(7.505, 0, 0);
	public static final PIDConstants AUTO_ROTATION_CONSTANTS = new PIDConstants(4.105, 0, 0);

	// Path following
	public static final PIDConstants PATH_FOLLOWING_TRANSLATION_CONSTANTS = new PIDConstants(6,0, 0);
	public static final PIDConstants PATH_FOLLOWING_ROTATION_CONSTANTS = new PIDConstants(2, 0, 0);
}