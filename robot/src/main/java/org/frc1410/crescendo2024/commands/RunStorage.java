package org.frc1410.crescendo2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2024.subsystems.Storage;


public class RunStorage extends Command {
	private final Storage storage;
	private final double RPM;

	public RunStorage(Storage storage, double RPM) {
		this.storage = storage;
		this.RPM = RPM;
		addRequirements(this.storage);
	}


	@Override
	public void initialize() {
		storage.setRPM(RPM);
	}

	@Override
	public boolean isFinished() {return false;}

	@Override
	public void end(boolean interrupted) {storage.setRPM(0);}
}
