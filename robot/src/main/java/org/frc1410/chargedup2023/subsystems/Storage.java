package org.frc1410.chargedup2023.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static org.frc1410.chargedup2023.util.IDs.*;


public class Storage implements Subsystem {
	private final CANSparkMax storageFrontLeft = new CANSparkMax(SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax storageFrontRight = new CANSparkMax(SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless);

	public Storage() {
		storageFrontLeft.setInverted(true);
		storageFrontRight.setInverted(false);

		storageFrontLeft.restoreFactoryDefaults();
		storageFrontRight.restoreFactoryDefaults();
	}

	public void setSpeed(double speed) {
		storageFrontLeft.set(speed);
		storageFrontRight.set(speed);

	}



}

