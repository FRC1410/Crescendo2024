package org.frc1410.crescendo2024.commands.shooterCommands;

import org.frc1410.crescendo2024.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2024.subsystems.Storage;

public class RunShooterLooped extends Command {
	private final Shooter shooter;
	private final Storage storage;
	private final double RPM;
	private final double storageRPM;

	public RunShooterLooped(Shooter shooter, Storage storage,double RPM, double storageRPM) {
		this.shooter = shooter;
		this.storage = storage;
		this.RPM = RPM;
		this.storageRPM = storageRPM;
		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		shooter.setRPM(RPM);

		if(shooter.getSpeed() > 3200) {
			storage.setRPM(1200);
		} else {
			storage.setRPM(0);
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		shooter.setRPM(0);
		storage.setRPM(0);
	}
}