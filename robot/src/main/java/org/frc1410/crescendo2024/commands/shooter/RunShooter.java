package org.frc1410.crescendo2024.commands.shooter;

import static edu.wpi.first.units.Units.RPM;

import org.frc1410.crescendo2024.subsystems.Shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;

public class RunShooter extends Command {
	private final Shooter shooter;

	private final Measure<Velocity<Angle>> velocity;

	public RunShooter(Shooter shooter, Measure<Velocity<Angle>> velocity) {
		this.shooter = shooter;
		this.velocity = velocity;

		this.addRequirements(shooter);
	}

	// TODO: add back adjustment
	@Override
	public void initialize() {
		this.shooter.setVelocity(this.velocity);
	}

	@Override
	public void execute() {
		this.shooter.setVelocity(this.velocity);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		this.shooter.setVelocity(RPM.zero());
	}
}