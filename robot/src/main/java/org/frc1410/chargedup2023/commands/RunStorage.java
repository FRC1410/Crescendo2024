package org.frc1410.chargedup2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.subsystems.Storage;
import org.frc1410.framework.control.Axis;
import static org.frc1410.chargedup2023.util.Constants.*;


public class RunStorage extends Command {
	private final Storage storage;
	private final Boolean isReversed;

	public RunStorage(Storage storage, boolean isReveresed) {
		this.storage = storage;
		this.isReversed = isReveresed;

		addRequirements(this.storage);
	}

	@Override
	public void initialize() {

	}


	@Override
	public void execute() {
		if(isReversed) {
			storage.setSpeed(-STORAGE_SPEED);
		} else if(!isReversed) {
			storage.setSpeed(STORAGE_SPEED);
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}


	@Override
	public void end(boolean interrupted) {
		storage.setSpeed(0);
	}
}
