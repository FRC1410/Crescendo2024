package org.frc1410.crescendo2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static org.frc1410.crescendo2023.util.IDs.*;


public class Storage implements Subsystem {
	private final WPI_TalonSRX storageFrontLeft = new WPI_TalonSRX(STORAGE_FRONT_LEFT_MOTOR_ID);
	private final WPI_TalonSRX storageFrontRight = new WPI_TalonSRX(STORAGE_FRONT_RIGHT_MOTOR_ID);
	private final  WPI_TalonSRX storageBackLeft = new WPI_TalonSRX(STORAGE_BACK_LEFT_MOTOR_ID);
	private final WPI_TalonSRX storageBackRight = new WPI_TalonSRX(STORAGE_BACK_RIGHT_MOTOR_ID);

	public Storage() {
		storageFrontLeft.setInverted(true);
		storageFrontRight.setInverted(false);
		storageBackLeft.setInverted(true);
		storageBackRight.setInverted(false);
	}

	public void setSpeed(double speed) {
		storageFrontLeft.set(speed);
		storageFrontRight.set(speed);
		storageBackLeft.set(speed);
		storageBackRight.set(speed);
	}

}