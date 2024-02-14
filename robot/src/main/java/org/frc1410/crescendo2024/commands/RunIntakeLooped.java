package org.frc1410.crescendo2024.commands;

import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.Storage;

import edu.wpi.first.wpilibj2.command.Command;

public class RunIntakeLooped extends Command {

	private final Intake intake;
	private final Storage storage;
	private final double intakeSpeed;
	private final double storageSpeed;

	public RunIntakeLooped(Intake intake, Storage storage, double intakeSpeed, double storageSpeed) {
		this.intake = intake;
		this.storage = storage;
		this.intakeSpeed = intakeSpeed;
		this.storageSpeed = storageSpeed;
		addRequirements(intake, storage);
	}

	@Override
	public void initialize() {
		intake.setSpeed(intakeSpeed);
		storage.setSpeed(storageSpeed);
//		storage.setRPM(storage.getStorageSpeed());
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		intake.setSpeed(0); 
		storage.setSpeed(0);
	}
}
