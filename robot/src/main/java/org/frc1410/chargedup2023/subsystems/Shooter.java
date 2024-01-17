package org.frc1410.chargedup2023.subsystems;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Subsystem;
import static org.frc1410.chargedup2023.util.IDs.*;




public class Shooter implements Subsystem {

	private final CANSparkMax shooterMotorRight = new CANSparkMax(SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax shooterMotorLeft = new CANSparkMax(SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);

	public Shooter() {
		shooterMotorLeft.restoreFactoryDefaults();
		shooterMotorRight.restoreFactoryDefaults();

		shooterMotorLeft.setInverted(true);
		shooterMotorRight.setInverted(false);
		//One Motor Will be Inverted (Double fly wheel)

		shooterMotorLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
		shooterMotorRight.setIdleMode(CANSparkBase.IdleMode.kBrake);
	}

	public void setSpeed(double speed) {
		shooterMotorLeft.set(speed);
		shooterMotorRight.set(speed);
	}
}
