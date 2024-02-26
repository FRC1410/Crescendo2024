package org.frc1410.crescendo2024.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static org.frc1410.crescendo2024.util.IDs.*;
import static org.frc1410.crescendo2024.util.Constants.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

public class Intake implements TickedSubsystem {

	private final CANSparkMax intakeMotorFront = new CANSparkMax(INTAKE_FRONT_MOTOR_ID, MotorType.kBrushless);

	private final CANSparkMax intakeMotorBack = new CANSparkMax(INTAKE_BACK_MOTOR_ID, MotorType.kBrushless);

	private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Drivetrain");

	private final DigitalInput lowerLimitSwitch = new DigitalInput(INTAKE_LIMIT_SWITCH_ID);

	public Intake () {
		intakeMotorFront.restoreFactoryDefaults();
		intakeMotorBack.restoreFactoryDefaults();

		intakeMotorFront.setInverted(INTAKE_FRONT_MOTOR_INVERTED);
		intakeMotorBack.setInverted(INTAKE_BACK_MOTOR_INVERTED);
		// One motor will be inverted (front) and the other will go in normal direction (opposite)

		intakeMotorFront.setIdleMode(CANSparkBase.IdleMode.kBrake);
		intakeMotorBack.setIdleMode(CANSparkBase.IdleMode.kBrake);
	}

	public void setSpeed(double speed) {
		intakeMotorFront.set(speed);
		intakeMotorBack.set(speed);
	}

	public boolean getLimitSwitch() {
		return !lowerLimitSwitch.get();
	}

	@Override
	public void periodic() {

	}
}

