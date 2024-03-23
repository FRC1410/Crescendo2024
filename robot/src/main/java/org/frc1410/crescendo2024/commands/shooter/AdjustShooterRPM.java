package org.frc1410.crescendo2024.commands.shooter;

import org.frc1410.crescendo2024.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class AdjustShooterRPM extends Command {
	private final Shooter shooter;

	private final double adjustment;

	public AdjustShooterRPM(Shooter shooter, double adjustment) {
		this.shooter = shooter;
		this.adjustment = adjustment;

		this.addRequirements(shooter);
	}

	@Override
	public void initialize() {
		this.shooter.rpmAdjustment += this.adjustment;

		System.out.println("RPM ADJUSTMENT: " + shooter.rpmAdjustment);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
