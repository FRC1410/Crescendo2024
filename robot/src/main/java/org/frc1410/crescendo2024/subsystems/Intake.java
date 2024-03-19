package org.frc1410.crescendo2024.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static org.frc1410.crescendo2024.util.IDs.*;
import static org.frc1410.crescendo2024.util.Constants.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import org.frc1410.crescendo2024.util.NetworkTables;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

public class Intake implements TickedSubsystem {

	private final CANSparkMax intakeMotorFront = new CANSparkMax(INTAKE_FRONT_MOTOR_ID, MotorType.kBrushless);

	private final CANSparkMax intakeMotorBack = new CANSparkMax(INTAKE_BACK_MOTOR_ID, MotorType.kBrushless);

	private final CANSparkMax intakeBarMotor = new CANSparkMax(INTAKE_BAR_MOTOR_ID, MotorType.kBrushless);

	private final CANSparkMax intakeExtendedMotor = new CANSparkMax(INTAKE_EXTENDED_MOTOR_ID, MotorType.kBrushless);

	private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Intake");

	private final DoublePublisher encoderCounter = NetworkTables.PublisherFactory(table, "encoder", 0);

	private final Encoder encoder = new Encoder(1, 2, true);
	private final DigitalInput lowerLimitSwitch = new DigitalInput(INTAKE_LIMIT_SWITCH_ID);

	private boolean isExtended = false;

	public Intake () {
		intakeMotorFront.restoreFactoryDefaults();
		intakeMotorBack.restoreFactoryDefaults();
		intakeBarMotor.restoreFactoryDefaults();
		intakeExtendedMotor.restoreFactoryDefaults();

		intakeMotorFront.setInverted(INTAKE_FRONT_MOTOR_INVERTED);
		intakeMotorBack.setInverted(INTAKE_BACK_MOTOR_INVERTED);
		intakeBarMotor.setInverted(INTAKE_BAR_MOTOR_INVERTED);
		intakeBarMotor.setInverted(INTAKE_EXTENDED_MOTOR_INVERTED);
		// One motor will be inverted (front) and the other will go in normal direction (opposite)

		intakeMotorFront.setIdleMode(CANSparkBase.IdleMode.kBrake);
		intakeMotorBack.setIdleMode(CANSparkBase.IdleMode.kBrake);

		intakeBarMotor.setInverted(true);


	}

	public void setSpeed(double speed) {
		intakeMotorFront.set(speed);
		intakeMotorBack.set(speed);
		intakeExtendedMotor.set(speed);
	}

	public void setExtended(boolean isExtended) {
		this.isExtended = isExtended;
	}

	public void extend(double speed) {
		intakeBarMotor.set(speed);
	}

	public boolean getLimitSwitch() {
		return !lowerLimitSwitch.get();
	}

	@Override
	public void periodic() {
		if (this.isExtended && this.encoder.get() < INTAKE_BAR_ENCODER_RANGE - 20) {
			this.intakeBarMotor.set(0.15);
		} else if (!this.isExtended && this.encoder.get() > 20) {
			this.intakeBarMotor.set(-0.15);
		} else {
			this.intakeBarMotor.set(0);
		}
		// System.out.println(encoder.get());
		encoderCounter.set(encoder.get());
	}
}