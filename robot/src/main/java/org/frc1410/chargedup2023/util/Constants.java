package org.frc1410.chargedup2023.util;

public interface Constants {
	// Controller constants
	int DRIVER_CONTROLLER = 0;
	int OPERATOR_CONTROLLER = 1;


	// Swerve module constants
	double DRIVING_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
	double DRIVE_ENCODER_CONSTANT = (1 / DRIVING_GEAR_RATIO) * 0.1016 * Math.PI;
	double MAX_ANGULAR_VEL = 25.57;
	double MAX_ANGULAR_ACC = 2 * Math.PI;  // radians per sec squared
	double MAX_SPEED = 4.2;
	double MAX_ANGULAR_SPEED = Math.PI;

	double FL_ANGLE_OFFSET = 27.9;
	double BL_ANGLE_OFFSET = 78.0;
	double FR_ANGLE_OFFSET = 89.3;
	double BR_ANGLE_OFFSET = 20.9;

	int WHEEL_RADIUS = 2;
	int SWERVE_MODULE_ENCODER_RES = 4095;


}

