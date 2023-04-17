package org.frc1410.chargedup2023.util;

public interface Tuning {

	// PID values for drive motor
	double SWERVE_DRIVE_KP = 0;
	double SWERVE_DRIVE_KI = 0;
	double SWERVE_DRIVE_KD = 0;

	// PID values for the turning motor
	double SWERVE_TURNING_KP = 0;
	double SWERVE_TURNING_KI = 0;
	double SWERVE_TURNING_KD = 0;

	// Speed, Velocity, and Acceleration values for swerve module
	double SWERVE_DRIVE_KS = 0;
	double SWERVE_DRIVE_KV = 0;
	double SWERVE_DRIVE_KA = 0;

	double TURN_KS = 0;
	double TURN_KV = 0;
	double TURN_KA = 0;
}