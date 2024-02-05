package org.frc1410.crescendo2024.commands;

import edu.wpi.first.wpilibj2.command.Command;

import static  org.frc1410.crescendo2024.util.Constants.*;

import org.frc1410.crescendo2024.subsystems.Shooter;


public class ShooterManual extends Command {
	private final Shooter shooter;

	public ShooterManual(Shooter shooter) {
		this.shooter = shooter;
		addRequirements(this.shooter);
	}

	@Override
	public void initialize() {

	}

	@Override
	public void execute() {

		shooter.setRPM(1);
		// TODO: write a way to check if shooter is sped up before shooting

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
