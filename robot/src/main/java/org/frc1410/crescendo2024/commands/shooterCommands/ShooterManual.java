package org.frc1410.crescendo2024.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;

import org.frc1410.crescendo2024.subsystems.Shooter;
import static org.frc1410.crescendo2024.util.Constants.*;

public class ShooterManual extends Command {
	private final Shooter shooter;
	private final double baseRpm;

	public ShooterManual(Shooter shooter, double baseRpm) {
		this.shooter = shooter;
		this.baseRpm = baseRpm;
		addRequirements(this.shooter);
	}

	@Override
	public void initialize() {
		shooter.setRPM(this.baseRpm + this.shooter.rpmAdjustment);
	}

	@Override
	public void execute() {
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
