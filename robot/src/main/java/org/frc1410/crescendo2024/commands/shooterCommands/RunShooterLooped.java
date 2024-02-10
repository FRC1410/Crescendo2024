package org.frc1410.crescendo2024.commands.shooterCommands;

import org.frc1410.crescendo2024.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class RunShooterLooped extends Command {
	private final Shooter shooter;
	private final double baseRpm;

	public RunShooterLooped(Shooter shooter, double baseRpm) {
		this.shooter = shooter;
		this.baseRpm = baseRpm;
		addRequirements(shooter);
	}

	@Override
	public void initialize() {
		shooter.setRPM(this.baseRpm + this.shooter.rpmAdjustment);
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