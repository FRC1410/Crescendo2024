package org.frc1410.crescendo2024.commands.shooterCommands;

import org.frc1410.crescendo2024.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class RunShooter extends Command {
	private final Shooter shooter;

	private final double rpm;

	public RunShooter(Shooter shooter, double rpm) {
		this.shooter = shooter;
		this.rpm = rpm;

		this.addRequirements(shooter);
	}

	@Override
	public void initialize() {
		this.shooter.setRPM(this.rpm + this.shooter.rpmAdjustment);
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