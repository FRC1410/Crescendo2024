package org.frc1410.crescendo2024.util;

import static org.frc1410.crescendo2024.util.Constants.*;

public interface Tuning {
    double SWERVE_DRIVE_P = 0.00001;
	double SWERVE_DRIVE_I = 0;
	double SWERVE_DRIVE_D = 0.0003;
	double SWERVE_DRIVE_FF = 0.00081 / DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND;

    double SWERVE_STEERING_P = 4;
	double SWERVE_STEERING_I = 0;
	double SWERVE_STEERING_D = 0;

	double LEFT_SHOOTER_RPM_MULT = 1;
	double RIGHT_SHOOTER_RPM_MULT = 1;
	double SHOOTER_LEFT_P = 0;
	double SHOOTER_LEFT_I = 0;
	double SHOOTER_LEFT_D = 0;
	double SHOOTER_LEFT_FF = 1;

	double SHOOTER_RIGHT_P = 0;
	double SHOOTER_RIGHT_I = 0;
	double SHOOTER_RIGHT_D = 0;
	double SHOOTER_RIGHT_FF = 1;

	double AMP_P = 0;
	double AMP_I = 0;
	double AMP_D = 0;
	double AMP_TOLERANCE = 0;

	// Auto PID Constants
	double AUTO_DRIVE_P = 0;
	double AUTO_DRIVE_I = 0;
	double AUTO_DRIVE_D = 0;

	double AUTO_TURN_P = 0;
	double AUTO_TURN_I = 0;
	double AUTO_TURN_D = 0;

}