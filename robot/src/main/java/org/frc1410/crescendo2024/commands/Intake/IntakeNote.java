package org.frc1410.crescendo2024.commands.Intake;

import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.Storage;

import edu.wpi.first.wpilibj2.command.Command;

import static org.frc1410.crescendo2024.util.Constants.INTAKE_SPEED;
import static org.frc1410.crescendo2024.util.Constants.STORAGE_INTAKE_RPM;

public class IntakeNote extends Command {
	private final Intake intake;
	private final Storage storage;

	private boolean limitSwitchAlreadyHit;

	public IntakeNote(Intake intake, Storage storage) {
		this.intake = intake;
		this.storage = storage;

		this.addRequirements(intake, storage);
	}

	@Override
	public void initialize() {
		if(this.intake.getLimitSwitch()) {
			this.limitSwitchAlreadyHit = true;
		} else {
			this.limitSwitchAlreadyHit = false;
		}

		this.intake.setExtended(true);
		this.intake.setSpeed(INTAKE_SPEED);
		this.storage.setRPM(STORAGE_INTAKE_RPM);
	}

	@Override
	public boolean isFinished() {
		return !this.limitSwitchAlreadyHit && this.intake.getLimitSwitch();
	}

	@Override
	public void end(boolean interrupted) {
		this.intake.setExtended(false);
		this.intake.setSpeed(0); 
		this.storage.setSpeed(0);
	}
}
