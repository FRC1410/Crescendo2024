package org.frc1410.chargedup2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc1410.chargedup2023.util.NetworkTables;
import org.frc1410.framework.scheduler.subsystem.TickedSubsystem;

import static org.frc1410.chargedup2023.util.IDs.*;
import static org.frc1410.chargedup2023.util.Tuning.*;
import static org.frc1410.chargedup2023.util.Constants.*;

public class AmpBar implements TickedSubsystem {

	private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Amp");

	private final CANSparkMax ampBarMotor = new CANSparkMax(AMP_BAR_MOTOR, MotorType.kBrushless);

	private final SparkPIDController ampPIDController = ampBarMotor.getPIDController();

	private final RelativeEncoder ampEncoder = ampBarMotor.getEncoder();

	private final DoublePublisher ampEncoderPosition = NetworkTables.PublisherFactory(table, "amp encoder pos", 0);

	public double pos;
	public boolean extended;

	public AmpBar() {
		ampBarMotor.restoreFactoryDefaults();
		resetPosition();
	}

	public double getPosition() {return ampEncoder.getPosition();}

	public void resetPosition() {ampEncoder.setPosition(AMP_HOME_POS);}

	public void setDesiredPosition(int DesiredPos) {ampEncoder.setPosition(DesiredPos);}

	@Override
	public void periodic() {
		ampEncoderPosition.set(getPosition());

	}
}


