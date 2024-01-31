package org.frc1410.chargedup2023.commands;


import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.subsystems.Intake;
import org.frc1410.chargedup2023.subsystems.Storage;
import org.frc1410.framework.control.Axis;
import static org.frc1410.chargedup2023.util.Constants.*;

public class RunIntakeLooped extends Command {
	private final Intake intake;
	private final Storage storage;
	private final Axis rightTrigger;
	private final Axis leftTrigger;

	public RunIntakeLooped(Intake intake, Storage storage, Axis rightTrigger, Axis leftTrigger) {
		this.intake = intake;
		this.storage = storage;
		this.rightTrigger = rightTrigger;
		this.leftTrigger = leftTrigger;

		addRequirements(intake, storage);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		if (rightTrigger.get() > .5) {
			intake.setSpeed(INTAKE_SPEED);
			storage.setSpeed(STORAGE_INTAKE_SPEED);
		} else if(leftTrigger.get() > .5) {
			intake.setSpeed(OUTTAKE_SPEED);
			intake.setSpeed(STORAGE_OUTTAKE_SPEED);
		} else {
			intake.setSpeed(0);
			storage.setSpeed(0);
		}
	}

	@Override
	public boolean isFinished() {return false;}
}
