package org.frc1410.crescendo2023.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.CAN;
 prelim-mechanism
import edu.wpi.first.wpilibj.DigitalInput;
=======
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
 prelim-intake
import edu.wpi.first.wpilibj2.command.Subsystem;
import static org.frc1410.crescendo2023.util.IDs.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



public class Intake implements Subsystem {
prelim-mechanism
	private final CANSparkMax intakeMotorFront = new CANSparkMax(INTAKE_FRONT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax intakeMotorBack = new CANSparkMax(INTAKE_BACK_MOTOR_ID, MotorType.kBrushless);
	DigitalInput intakeLimitSwitch = new DigitalInput(INTAKE_LIMIT_SWITCH);

=======
	private final WPI_TalonSRX intakeMotorFront = new WPI_TalonSRX(INTAKE_FRONT_MOTOR_ID);
	private final WPI_TalonSRX intakeMotorBack = new WPI_TalonSRX(INTAKE_BACK_MOTOR_ID);
 prelim-intake
	public Intake () {


		intakeMotorFront.setInverted(false);
		intakeMotorBack.setInverted(false);
		// One motor will be inverted (front) and the other will go in normal direction (opposite)

 prelim-mechanism
		intakeMotorFront.setIdleMode(CANSparkBase.IdleMode.kBrake);
		intakeMotorBack.setIdleMode(CANSparkBase.IdleMode.kBrake);
		if(intakeLimitSwitch.get()){
			setSpeed(0);
		}
=======
prelim-intake
	}



	public void setSpeed(double speed) {
		intakeMotorFront.set(speed);
		intakeMotorBack.set(speed);

	}
}
