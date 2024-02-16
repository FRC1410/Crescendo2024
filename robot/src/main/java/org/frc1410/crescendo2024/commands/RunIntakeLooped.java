package org.frc1410.crescendo2024.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.Storage;

import edu.wpi.first.wpilibj2.command.Command;

public class RunIntakeLooped extends Command {

	private final Intake intake;
	private final Storage storage;
	private final double intakeSpeed;
	private final double storageSpeed;

	private boolean limitSwitchHit;

	public RunIntakeLooped(Intake intake, Storage storage, double intakeSpeed, double storageSpeed) {
		this.intake = intake;
		this.storage = storage;
		this.intakeSpeed = intakeSpeed;
		this.storageSpeed = storageSpeed;

		addRequirements(intake, storage);
	}

	@Override
	public void initialize() {
		limitSwitchHit = false;
	}

	@Override
	public void execute() {
		intake.setSpeed(intakeSpeed);
		storage.setRPM(storageSpeed);

		if(intake.getLimitSwitch()) {
			intake.setSpeed(0);
			storage.setRPM(0);

			limitSwitchHit = true;
		} else {
			limitSwitchHit = false;
		}
//		if(!intake.getLimitSwitch()) {
//			limitSwitchHit = false;
//		}
	}


	@Override
	public boolean isFinished() {
        return limitSwitchHit;
	}

	@Override
	public void end(boolean interrupted) {
		intake.setSpeed(0); 
		storage.setSpeed(0);
	}

}
