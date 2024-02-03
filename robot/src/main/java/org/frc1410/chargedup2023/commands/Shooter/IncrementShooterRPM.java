package org.frc1410.chargedup2023.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.chargedup2023.subsystems.Shooter;

public class IncrementShooterRPM extends Command {
	private final Shooter shooter;
	private final double increment;

	public IncrementShooterRPM(Shooter shooter, double increment) {
		this.shooter = shooter;
		this.increment = increment;
		addRequirements(this.shooter);
	}

	@Override
	public void initialize() {
		shooter.rpmAdjustment += increment;
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
