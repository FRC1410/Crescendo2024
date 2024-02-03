package org.frc1410.chargedup2023.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static org.frc1410.chargedup2023.util.IDs.*;
import static org.frc1410.chargedup2023.util.Constants.*;


public class Storage implements Subsystem {

	private final CANSparkMax storageLeftMotor = new CANSparkMax(STORAGE_LEFT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax storageRightMotor = new CANSparkMax(STORAGE_RIGHT_MOTOR_ID, MotorType.kBrushless);

	public Storage() {
		storageLeftMotor.setInverted(STORAGE_LEFT_MOTOR_INVERTED);
		storageRightMotor.setInverted(STORAGE_RIGHT_MOTOR_INVERTED);

		storageLeftMotor.restoreFactoryDefaults();
		storageRightMotor.restoreFactoryDefaults();
	}

	public void setSpeed(double speed) {
		storageLeftMotor.set(speed);
		storageRightMotor.set(speed);

	}
}