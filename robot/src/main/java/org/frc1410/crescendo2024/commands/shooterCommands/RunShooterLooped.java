package org.frc1410.crescendo2024.commands.shooterCommands;

import org.frc1410.crescendo2024.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc1410.crescendo2024.subsystems.Storage;

public class RunShooterLooped extends Command {
	private final Shooter shooter;
	private final double shooterRPM;

	public RunShooterLooped(Shooter shooter, double shooterRPM) {
		this.shooter = shooter;
		this.shooterRPM = shooterRPM;
		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		shooter.setRPM(shooterRPM + shooter.rpmAdjustment);
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