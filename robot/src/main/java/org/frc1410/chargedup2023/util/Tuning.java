package org.frc1410.chargedup2023.util;

import static org.frc1410.chargedup2023.util.Constants.*;

public interface Tuning {

	// PID values for drive motor
	
	double SWERVE_DRIVE_KP = 0.00001;
	// double SWERVE_DRIVE_KP = 0;
	double SWERVE_DRIVE_KI = 0;
	double SWERVE_DRIVE_KD = 0.0003;
	double SWERVE_DRIVE_KFF = 0.00081 / DRIVE_WHEEL_FREE_SPEED_METERS_PER_SECOND;

	// PID values for the steering motor
	double SWERVE_STEERING_KP = 4;
	double SWERVE_STEERING_KI = 0;
	double SWERVE_STEERING_KD = 0;
	// double SWERVE_STEERING_KP = 0;
	// double SWERVE_STEERING_KI = 0;
	// double SWERVE_STEERING_KD = 0;

	// Speed, Velocity, and Acceleration values for swerve module
	double SWERVE_DRIVE_KS = 1;
	double SWERVE_DRIVE_KV = 2.5;
	// double SWERVE_DRIVE_KA = 0;

	// double STEER_KS = 0;
	// double STEER_KV = 0.38;
	// double STEER_KA = 0;
}