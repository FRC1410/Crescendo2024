package org.frc1410.crescendo2024.commands.Intake;

import org.frc1410.crescendo2024.subsystems.Intake;
import org.frc1410.crescendo2024.subsystems.LEDs;
import org.frc1410.crescendo2024.subsystems.Storage;

import edu.wpi.first.wpilibj2.command.Command;

import static org.frc1410.crescendo2024.subsystems.LEDs.Colors.OCEAN_BREEZE;
import static org.frc1410.crescendo2024.subsystems.LEDs.Colors.PRANCING_PONY_PINK;

public class RunIntakeLimitSwitch extends Command {

	private final Intake intake;
	private final Storage storage;
	private LEDs leds = new LEDs();

	private final double intakeSpeed;
	private final double storageSpeed;
	private boolean limitSwitchHit;
	private boolean limitSwitchAlreadyHit;

	public RunIntakeLimitSwitch(Intake intake, Storage storage, double intakeSpeed, double storageSpeed) {
		this.intake = intake;
		this.storage = storage;
		this.intakeSpeed = intakeSpeed;
		this.storageSpeed = storageSpeed;

		addRequirements(intake, storage, leds);
	}

	@Override
	public void initialize() {

		limitSwitchHit = false;
		if(intake.getLimitSwitch()) {
			limitSwitchAlreadyHit = true;
		} else {
			limitSwitchAlreadyHit = false;
		}

		this.intake.setExtended(true);
	}

	@Override
	public void execute() {

		if(limitSwitchAlreadyHit) {
			intake.setSpeed(intakeSpeed);
			storage.setRPM(storageSpeed);
		} else {
			intake.setSpeed(intakeSpeed);
			storage.setRPM(storageSpeed);
			if (intake.getLimitSwitch()) {
				intake.setSpeed(0);
				storage.setRPM(0);

				leds.changeLEDsColor(PRANCING_PONY_PINK);

				limitSwitchHit = true;
			} else {
				leds.changeLEDsColor(OCEAN_BREEZE);
				limitSwitchHit = false;
			}
		}
	}


	@Override
	public boolean isFinished() {
        return limitSwitchHit;
	}

	@Override
	public void end(boolean interrupted) {
		intake.setSpeed(0); 
		storage.setSpeed(0);
		this.intake.setExtended(false);
	}

}
