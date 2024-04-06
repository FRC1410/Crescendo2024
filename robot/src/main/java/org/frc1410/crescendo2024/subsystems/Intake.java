package org.frc1410.crescendo2024.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

import static org.frc1410.crescendo2024.util.IDs.*;
import static org.frc1410.crescendo2024.util.Constants.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

public class Intake implements TickedSubsystem {
	private final CANSparkMax frontMotor = new CANSparkMax(INTAKE_FRONT_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax backMotor = new CANSparkMax(INTAKE_BACK_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax extendedMotor = new CANSparkMax(INTAKE_EXTENDED_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax barMotor = new CANSparkMax(INTAKE_BAR_MOTOR_ID, MotorType.kBrushless);

	private final Encoder barEncoder = new Encoder(INTAKE_BAR_ENCODER_CHANNEL_A, INTAKE_BAR_ENCODER_CHANNEL_B, true);

	private final DigitalInput limitSwitch = new DigitalInput(INTAKE_LIMIT_SWITCH_ID);

	private boolean isExtended = false;

	private double barEncoderOffset = 0;

	public Intake() {
		this.frontMotor.restoreFactoryDefaults();
		this.backMotor.restoreFactoryDefaults();
		this.extendedMotor.restoreFactoryDefaults();
		this.barMotor.restoreFactoryDefaults();

		this.frontMotor.setInverted(INTAKE_FRONT_MOTOR_INVERTED);
		this.backMotor.setInverted(INTAKE_BACK_MOTOR_INVERTED);
		this.extendedMotor.setInverted(INTAKE_EXTENDED_MOTOR_INVERTED);
		this.barMotor.setInverted(INTAKE_BAR_MOTOR_INVERTED);

		this.frontMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
		this.backMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
		this.extendedMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
		this.barMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

		this.barMotor.setSmartCurrentLimit(30);
	}

	public void setSpeed(double speed) {
		this.frontMotor.set(speed);
		this.backMotor.set(speed);
		this.extendedMotor.set(speed);
	}

	public void setExtended(boolean isExtended) {
//		this.isExtended = isExtended;
		this.isExtended = false;
	}

	public void extend(double speed) {
		this.barMotor.set(speed);
	}

	public boolean isExtended() {
		return this.isExtended;
	}

	public boolean getLimitSwitch() {
		return !this.limitSwitch.get();
	}

	public void zeroBarEncoder() {
		this.barEncoderOffset = -INTAKE_BAR_ENCODER_RANGE;
	}

	private double getBarEncoder() {
		return this.barEncoder.get() - this.barEncoderOffset;
	}

	@Override
	public void periodic() {
		if (this.isExtended && this.getBarEncoder() < INTAKE_BAR_ENCODER_RANGE - 20) {
			this.barMotor.set(INTAKE_BAR_SPEED_DOWN);
		} else if (!this.isExtended && this.getBarEncoder() > 20) {
			this.barMotor.set(-INTAKE_BAR_SPEED_UP);
		} else {
			this.barMotor.set(0);
		}
	}
}