package org.frc1410.crescendo2024.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;

import org.frc1410.crescendo2024.subsystems.Shooter;


public class ShooterManual extends Command {
	private final Shooter shooter;

	public ShooterManual(Shooter shooter) {
		this.shooter = shooter;
		addRequirements(this.shooter);
	}

	@Override
	public void initialize() {
//		shooter.setRPM(shooter.getSpeed());
		shooter.setRPM(shooter.getSpeed() + shooter.rpmAdjustment);
	}

	@Override
	public void execute() {}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		shooter.setRPM(0);
	}
}
