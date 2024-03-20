package org.frc1410.crescendo2024.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2024.subsystems.Storage;

public class RunStorage extends Command {
	private final Storage storage;

	private final double rpm;

	public RunStorage(Storage storage, double rpm) {
		this.storage = storage;
		this.rpm = rpm;

		this.addRequirements(this.storage);
	}

	@Override
	public void initialize() {
		this.storage.setRPM(this.rpm);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.storage.setRPM(0);
	}
}
