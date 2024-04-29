package org.frc1410.crescendo2024.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

import static org.frc1410.crescendo2024.util.IDs.*;

import org.frc1410.crescendo2024.util.NetworkTables;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static org.frc1410.crescendo2024.util.Constants.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

public class Intake implements TickedSubsystem {
	private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Intake");

    private final DoublePublisher barEncoderAngle = NetworkTables.PublisherFactory(this.table, "Bar Encoder Angle", 0);
	private final BooleanPublisher hasNote = NetworkTables.PublisherFactory(this.table, "Has Note", false);

	private final CANSparkMax sushiRollerMotor = new CANSparkMax(INTAKE_SUSHI_ROLLER_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax innerMotor = new CANSparkMax(INTAKE_INNER_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax outerMotor = new CANSparkMax(INTAKE_OUTER_MOTOR_ID, MotorType.kBrushless);
	private final CANSparkMax barMotor = new CANSparkMax(INTAKE_BAR_MOTOR_ID, MotorType.kBrushless);

	private final Encoder barEncoder = new Encoder(INTAKE_BAR_ENCODER_CHANNEL_A, INTAKE_BAR_ENCODER_CHANNEL_B, true);

	private final DigitalInput limitSwitch = new DigitalInput(INTAKE_LIMIT_SWITCH_ID);

	private boolean isExtended = false;

	private Measure<Angle> barEncoderOffset = Degrees.zero();

	public Intake() {
		this.sushiRollerMotor.restoreFactoryDefaults();
		this.innerMotor.restoreFactoryDefaults();
		this.outerMotor.restoreFactoryDefaults();
		this.barMotor.restoreFactoryDefaults();

		// TODO: rename constants
		this.sushiRollerMotor.setInverted(INTAKE_SUSHI_ROLLER_MOTOR_INVERTED);
		this.innerMotor.setInverted(INTAKE_INNER_MOTOR_INVERTED);
		this.outerMotor.setInverted(INTAKE_OUTER_MOTOR_INVERTED);
		this.barMotor.setInverted(INTAKE_BAR_MOTOR_INVERTED);

		this.sushiRollerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
		this.innerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
		this.outerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
		this.barMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

		this.sushiRollerMotor.setSmartCurrentLimit(30);
		this.innerMotor.setSmartCurrentLimit(30);
		this.outerMotor.setSmartCurrentLimit(30);
		this.barMotor.setSmartCurrentLimit(30);
	}

	public void setSpeed(double speed) {
		this.sushiRollerMotor.set(speed);
		this.innerMotor.set(speed * 4/3);
		this.outerMotor.set(speed);
	}

	public void setUnderBumperSpeed(double speed) {
		this.sushiRollerMotor.set(speed);
		this.innerMotor.set(speed * 4/3);
	}

	public void setExtended(boolean isExtended) {
		this.isExtended = isExtended;
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
		this.barEncoderOffset = INTAKE_BAR_ENCODER_RANGE.negate();
	}

	private Measure<Angle> getBarEncoder() {
		return Rotations.of(
			((double) this.barEncoder.get()) / 2048
		)
		.minus(this.barEncoderOffset);
	}

	@Override
	public void periodic() {
		if (this.isExtended && this.getBarEncoder().lt(INTAKE_BAR_ENCODER_RANGE.minus(Degrees.of(3.5)))) {
			this.barMotor.set(INTAKE_BAR_SPEED_DOWN);
		} else if (!this.isExtended && this.getBarEncoder().gt(Degrees.of(3.5))) {
			this.barMotor.set(-INTAKE_BAR_SPEED_UP);
		} else {
			this.barMotor.set(0);
		}

		this.barEncoderAngle.set(this.getBarEncoder().in(Degrees));
		this.hasNote.set(this.getLimitSwitch());
	}
}