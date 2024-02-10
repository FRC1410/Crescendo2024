package org.frc1410.crescendo2024.util;

import static org.frc1410.crescendo2024.util.Constants.*;

public interface Tuning {
    double SWERVE_DRIVE_KP = 0.00001;
	double SWERVE_DRIVE_KI = 0;
	double SWERVE_DRIVE_KD = 0.0003;
	double SWERVE_DRIVE_KFF = 0.00081 / DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND;

    double SWERVE_STEERING_KP = 4;
	double SWERVE_STEERING_KI = 0;
	double SWERVE_STEERING_KD = 0;

	double LEFT_SHOOTER_RPM_MULT = 1;
	double RIGHT_SHOOTER_RPM_MULT = 1;
	double SHOOTER_LEFT_KP = 0;
	double SHOOTER_LEFT_KI = 0;
	double SHOOTER_LEFT_KD = 0;
	double SHOOTER_LEFT_KFF = 1;

	double SHOOTER_RIGHT_KP = 0;
	double SHOOTER_RIGHT_KI = 0;
	double SHOOTER_RIGHT_KD = 0;
	double SHOOTER_RIGHT_KFF = 1;

	double AMP_KP = 0;
	double AMP_KI = 0;
	double AMP_KD = 0;
	double AMP_TOLERANCE = 0;
}