package org.frc1410.chargedup2023.util;

public interface Tuning {

	// PID values for drive motor
	double SWERVE_DRIVE_KP = 0.08;
	double SWERVE_DRIVE_KI = 0;
	double SWERVE_DRIVE_KD = 0;

	// PID values for the steering motor
	double SWERVE_STEERING_KP = 4;
	double SWERVE_STEERING_KI = 0;
	double SWERVE_STEERING_KD = 0;
	// double SWERVE_STEERING_KP = 0;
	// double SWERVE_STEERING_KI = 0;
	// double SWERVE_STEERING_KD = 0;

	// Speed, Velocity, and Acceleration values for swerve module
	double SWERVE_DRIVE_KS = 1;
	double SWERVE_DRIVE_KV = 3;
	// double SWERVE_DRIVE_KA = 0;

	double STEER_KS = 0;
	double STEER_KV = 0.38;
	// double STEER_KA = 0;
}