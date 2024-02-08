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

		this.climbLeft.getPIDController().setP(CLIMB_LEFT_P);
		this.climbLeft.getPIDController().setI(CLIMB_LEFT_I);
		this.climbLeft.getPIDController().setD(CLIMB_LEFT_D);
		this.climbLeft.getPIDController().setFF(CLIMB_LEFT_FF);

		this.climbRight.getPIDController().setP(CLIMB_RIGHT_P);
		this.climbRight.getPIDController().setI(CLIMB_RIGHT_I);
		this.climbRight.getPIDController().setD(CLIMB_RIGHT_D);
		this.climbRight.getPIDController().setFF(CLIMB_RIGHT_FF);


	}

	public void setSpeed(double speed) {

		leftPIDController.setReference(speed, CANSparkBase.ControlType.kPosition);
		rightPIDController.setReference(speed, CANSparkBase.ControlType.kPosition);

	}
}
