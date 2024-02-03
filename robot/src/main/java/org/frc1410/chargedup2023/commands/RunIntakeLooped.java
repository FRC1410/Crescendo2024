package org.frc1410.chargedup2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.subsystems.Intake;
import org.frc1410.chargedup2023.subsystems.Storage;

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
		addRequirements( intake, storage);
	}

	@Override
	public void initialize() {
		intake.setSpeed(intakeSpeed);
		storage.setSpeed(storageSpeed);
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
