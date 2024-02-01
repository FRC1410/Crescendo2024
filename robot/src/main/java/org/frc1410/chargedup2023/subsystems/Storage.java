package org.frc1410.chargedup2023.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static org.frc1410.chargedup2023.util.IDs.*;


public class Storage implements Subsystem {
	private final WPI_TalonSRX storageFrontLeft = new WPI_TalonSRX(STORAGE_FRONT_LEFT_MOTOR_ID);
	private final WPI_TalonSRX storageFrontRight = new WPI_TalonSRX(STORAGE_FRONT_RIGHT_MOTOR_ID);
	private final  WPI_TalonSRX storageBackLeft = new WPI_TalonSRX(STORAGE_BACK_LEFT_MOTOR_ID);
	private final WPI_TalonSRX storageBackRight = new WPI_TalonSRX(STORAGE_BACK_RIGHT_MOTOR_ID);
	DigitalInput firstStorageLimitSwitch = new DigitalInput(STORAGE_LIMIT_SWITCH_ID);
	DigitalInput intakeLimitSwitch = new DigitalInput(SECOND_LIMIT_SWTICH_ID);


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


	public boolean getLimitSwitch() {
		return intakeLimitSwitch.get();
	}
	public boolean getStorageSwitch() {
		return firstStorageLimitSwitch.get();
	}
}

