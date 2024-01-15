package org.frc1410.crescendo2023.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static org.frc1410.crescendo2023.util.IDs.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



public class Intake implements Subsystem {
	private final WPI_TalonSRX intakeMotorFront = new WPI_TalonSRX(INTAKE_FRONT_MOTOR_ID);
	private final WPI_TalonSRX intakeMotorBack = new WPI_TalonSRX(INTAKE_BACK_MOTOR_ID);
	public Intake () {


		intakeMotorFront.setInverted(false);
		intakeMotorBack.setInverted(false);
		// One motor will be inverted (front) and the other will go in normal direction (opposite)

	}

	public void setSpeed(double speed) {
		intakeMotorFront.set(speed);
		intakeMotorBack.set(speed);

	}
}
