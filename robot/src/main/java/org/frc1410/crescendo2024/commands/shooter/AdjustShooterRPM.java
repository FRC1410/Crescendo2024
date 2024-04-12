package org.frc1410.crescendo2024.commands.shooter;

import static edu.wpi.first.units.Units.RPM;

import org.frc1410.crescendo2024.subsystems.Shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;

public class AdjustShooterRPM extends Command {
	private final Shooter shooter;

	private final Measure<Velocity<Angle>> adjustment;

	public AdjustShooterRPM(Shooter shooter, Measure<Velocity<Angle>> adjustment) {
		this.shooter = shooter;
		this.adjustment = adjustment;

		this.addRequirements(shooter);
	}

	@Override
	public void initialize() {
		this.shooter.velocityAdjustment = this.shooter.velocityAdjustment.plus(this.adjustment);

		System.out.println("RPM ADJUSTMENT: " + shooter.velocityAdjustment.in(RPM));
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
