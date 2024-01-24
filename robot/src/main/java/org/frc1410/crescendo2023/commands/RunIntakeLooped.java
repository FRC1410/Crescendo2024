package org.frc1410.crescendo2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2023.subsystems.Intake;
import org.frc1410.crescendo2023.subsystems.Storage;
import org.frc1410.framework.control.Axis;
import static org.frc1410.crescendo2023.util.Constants.*;


public class RunIntakeLooped extends Command {
	private final Intake intake;
	 boolean isReversed;
	 private final Storage storage;

	public RunIntakeLooped(Intake intake, Storage storage, boolean isReversed ) {
		this.intake = intake;
		this.isReversed = isReversed;
		this.storage = storage;

		addRequirements(intake,storage);
	}


	@Override
	public void initialize() {

	}


	@Override
	public void execute() {
		if(isReversed) {
			intake.setSpeed(-INTAKE_SPEED);
			storage.setSpeed(-STORAGE_SPEED);
		}
		if(!isReversed) {
			intake.setSpeed(INTAKE_SPEED);
			storage.setSpeed(STORAGE_SPEED);
		}
	}


	@Override
	public boolean isFinished() {
		// TODO: Make this return true when this Command no longer needs to run execute()
		return false;
	}


	@Override
	public void end(boolean interrupted) {
		intake.setSpeed(0);
		storage.setSpeed(0);

	}
}
