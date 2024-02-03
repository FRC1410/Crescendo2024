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

public class AmpBar implements Subsystem {

	private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Amp");

	private final CANSparkMax ampBarMotor = new CANSparkMax(AMP_BAR_MOTOR_ID, MotorType.kBrushless);

	private final SparkPIDController ampPIDController = ampBarMotor.getPIDController();


	public AmpBar() {
		ampBarMotor.setInverted(AMP_BAR_MOTOR_INVERTED);
		ampBarMotor.restoreFactoryDefaults();
	}

	public void setSpeed(double speed) {
		ampBarMotor.set(speed);
	}

}


