package org.frc1410.chargedup2023.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static org.frc1410.chargedup2023.util.IDs.*;
import static org.frc1410.chargedup2023.util.Tuning.*;

public class Climb implements Subsystem{
	private final CANSparkMax climbLeft = new CANSparkMax(CLIMB_LEFT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
	private final CANSparkMax climbRight = new CANSparkMax(CLIMB_RIGHT_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

	//lol
	private final SparkPIDController leftPIDController = climbLeft.getPIDController();
	private final SparkPIDController rightPIDController = climbLeft.getPIDController();
	public Climb() {
		climbLeft.restoreFactoryDefaults();
		climbRight.restoreFactoryDefaults();

		climbLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
		climbRight.setIdleMode(CANSparkBase.IdleMode.kBrake);

		climbRight.setInverted(true);

		this.leftPIDController.setP(CLIMB_LEFT_KP);
		this.leftPIDController.setP(CLIMB_LEFT_KI);
		this.leftPIDController.setP(CLIMB_LEFT_KD);
		this.leftPIDController.setP(CLIMB_LEFT_KFF);
		this.rightPIDController.setP(CLIMB_RIGHT_KP);
		this.rightPIDController.setP(CLIMB_RIGHT_KI);
		this.rightPIDController.setP(CLIMB_RIGHT_KD);
		this.rightPIDController.setP(CLIMB_RIGHT_KFF);


	}

	//I hate my life

	public void setSpeed(double speed) {
		leftPIDController.setReference(speed, CANSparkBase.ControlType.kPosition);
		rightPIDController.setReference(speed, CANSparkBase.ControlType.kPosition); //Soren Bad at Programming
	}
}
