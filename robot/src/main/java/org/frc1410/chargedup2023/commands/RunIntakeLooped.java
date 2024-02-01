package org.frc1410.chargedup2023.commands;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.subsystems.Intake;
import org.frc1410.chargedup2023.subsystems.Storage;


import static org.frc1410.chargedup2023.util.Constants.*;


public class RunIntakeLooped extends Command {
	private final Intake intake;
	private final Storage storage;
	Storage storageBackRight;
	Storage storageBackLeft;
	Storage storageFrontRight;
	Storage storageFrontLeft;
	boolean isReversed;
	DigitalInput firstStoragelimitSwtich;


	public RunIntakeLooped(Intake intake, Storage storage, boolean isReveresed) {
		this.intake = intake;
		this.storage = storage;
		this.isReversed = isReveresed;


		addRequirements( intake, storage);
	}


	@Override
	public void initialize() {


	}


	@Override
	public void execute() {
		if (isReversed) {
			intake.setSpeed(INTAKE_SPEED);
			storage.setSpeed(STORAGE_SPEED);
		}
		if(storage.getLimitSwitch()){
			intake.setSpeed(0);
			storageFrontRight.setSpeed(0);
			storageFrontLeft.setSpeed(0);

		}

	}


	@Override
	public boolean isFinished() {
		// TODO: Make this return true when this Command no longer needs to run execute()
		return false;
	}
}
