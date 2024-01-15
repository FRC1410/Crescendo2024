package org.frc1410.crescendo2023.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static org.frc1410.crescendo2023.util.IDs.*;

public class Climb implements Subsystem {

	private final CANSparkMax climbLeft = new CANSparkMax(CLIMB_LEFT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax climbRight = new CANSparkMax(CLIMB_RIGHT_MOTOR_ID, MotorType.kBrushless);

	public Climb() {
		climbLeft.restoreFactoryDefaults();
		climbRight.restoreFactoryDefaults();

		climbLeft.setInverted(false);
		climbRight.setInverted(false);

		climbLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
		climbRight.setIdleMode(CANSparkBase.IdleMode.kBrake);
	}


	public void setSpeeds(double speeds) {
		climbLeft.set(speeds);
		climbRight.set(speeds);
	}

}
