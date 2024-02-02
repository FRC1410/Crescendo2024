package org.frc1410.chargedup2023.commands;


import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.subsystems.Intake;
import org.frc1410.chargedup2023.subsystems.Storage;


import static org.frc1410.chargedup2023.util.Constants.*;


public class RunIntakeLooped extends Command {

	private final Intake intake;
	private final Storage storage;
	double intakeSpeed;
	double storageSpeed;
	int isReversed;

	public RunIntakeLooped(Intake intake, Storage storage, double intakeSpeed, double storageSpeed, int isReveresed) {
		this.intake = intake;
		this.storage = storage;
		this.isReversed = isReveresed;
		this.intakeSpeed = intakeSpeed;
		this.storageSpeed = storageSpeed;
		addRequirements( intake, storage);
	}

	@Override
	public void execute() {
		intake.setSpeed(intakeSpeed * isReversed);
		storage.setSpeed(storageSpeed * isReversed);
	}

	@Override
	public boolean isFinished() {
		// TODO: Make this return true when this Command no longer needs to run execute()
		return false;
	}
}
