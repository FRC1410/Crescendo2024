package org.frc1410.chargedup2023.util;

public interface Constants {
	// Controller constants
	int DRIVER_CONTROLLER = 0;
	int OPERATOR_CONTROLLER = 1;


	// Swerve module constants
	double DRIVING_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
	double MAX_ANGULAR_VEL = 25.57;
	double MAX_ANGULAR_ACC = 2 * Math.PI;  // radians per sec squared

	int WHEEL_RADIUS = 2;
	int SWERVE_MODULE_ENCODER_RES = 4095;
}

