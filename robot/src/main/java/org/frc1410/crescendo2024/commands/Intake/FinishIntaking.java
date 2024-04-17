package org.frc1410.crescendo2024.commands.intake;

import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.Storage;

import edu.wpi.first.wpilibj2.command.Command;

import static org.frc1410.crescendo2024.util.Constants.INTAKE_SPEED;
import static org.frc1410.crescendo2024.util.Constants.STORAGE_INTAKE_VELOCITY;

public class FinishIntaking extends Command {
	private final Intake intake;
	private final Storage storage;

	public FinishIntaking(Intake intake, Storage storage) {
		this.intake = intake;
		this.storage = storage;

		this.addRequirements(intake, storage);
	}

	@Override
	public void initialize() {
		this.intake.setSpeed(INTAKE_SPEED);
		this.storage.setVelocity(STORAGE_INTAKE_VELOCITY);
	}

	@Override
	public boolean isFinished() {
		return this.intake.getLimitSwitch();
	}

	@Override
	public void end(boolean interrupted) {
		this.intake.setSpeed(0); 
		this.storage.setSpeed(0);
	}
}
