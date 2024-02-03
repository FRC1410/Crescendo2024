package org.frc1410.chargedup2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.subsystems.Shooter;
import org.frc1410.chargedup2023.subsystems.Storage;

import static  org.frc1410.chargedup2023.util.Constants.*;

public class ScoreAmp extends Command {

	private final Shooter shooter;
	private final Storage storage;

	public ScoreAmp(Shooter shooter, Storage storage) {
		this.shooter = shooter;
		this.storage = storage;
		addRequirements(this.shooter);
	}

	@Override
	public void execute() {
		shooter.setRPM(AMP_SHOOT_SPEED);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		shooter.setRPM(0);
	}
}
