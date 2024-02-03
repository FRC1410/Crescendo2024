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
}