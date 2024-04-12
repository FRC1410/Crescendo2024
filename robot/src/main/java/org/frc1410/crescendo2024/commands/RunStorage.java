package org.frc1410.crescendo2024.commands;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.RPM;

import org.frc1410.crescendo2024.subsystems.Storage;

public class RunStorage extends Command {
	private final Storage storage;

	private final Measure<Velocity<Angle>> velocity;

	public RunStorage(Storage storage, Measure<Velocity<Angle>> velocity) {
		this.storage = storage;
		this.velocity = velocity;

		this.addRequirements(this.storage);
	}

	@Override
	public void initialize() {
		this.storage.setVelocity(this.velocity);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.storage.setVelocity(RPM.zero());
	}
}
