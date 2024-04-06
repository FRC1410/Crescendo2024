package org.frc1410.crescendo2024.util;

import com.pathplanner.lib.util.PIDConstants;

public interface Tuning {
	// Drivetrai 
	double SWERVE_DRIVE_P = 0;
	double SWERVE_DRIVE_I = 0;
	double SWERVE_DRIVE_D = 0;
	double SWERVE_DRIVE_FF = 0.00032;

    double SWERVE_STEERING_P = 4;
	double SWERVE_STEERING_I = 0;
	double SWERVE_STEERING_D = 0;

	// Shooter
	double SHOOTER_LEFT_P = 0;
	double SHOOTER_LEFT_I = 0;
	double SHOOTER_LEFT_D = 0;
	double SHOOTER_LEFT_FF = 1;

	double SHOOTER_RIGHT_P = 0;
	double SHOOTER_RIGHT_I = 0;
	double SHOOTER_RIGHT_D = 0;	
	double SHOOTER_RIGHT_FF = 1;

	// Auto
	// TODO: old 5, 4
	PIDConstants AUTO_TRANSLATION_CONSTANTS = new PIDConstants(6.5, 0, 0);
	PIDConstants AUTO_ROTATION_CONSTANTS = new PIDConstants(4.5, 0, 0);

	// Path following
	PIDConstants PATH_FOLLOWING_TRANSLATION_CONSTANTS = new PIDConstants(6,0, 0);
	PIDConstants PATH_FOLLOWING_ROTATION_CONSTANTS = new PIDConstants(2, 0, 0);
}